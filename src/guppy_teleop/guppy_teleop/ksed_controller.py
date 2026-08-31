from typing import Any

import pygame
import rclpy
from geometry_msgs.msg import Twist
from guppy_msgs.srv._change_state import ChangeState
from pygame.joystick import JoystickType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from guppy_msgs.msg import State

MULTIPLIER = 1

COLOR_FAULT = (255, 0, 0)
COLOR_DISABLED = (255, 255, 0)
COLOR_TELEOP = (0, 255, 0)
COLOR_STARTUP = (0, 200, 255)


# error class for RawController
class ControllerError(Exception):
    def __init__(self, msg="There was an issue with the controller."):
        super().__init__(msg)


# manages single controller input
class RawController:
    def __init__(self, name: str) -> None:
        self._joystick = None

        if not name:
            raise ControllerError("Controller must have a name.")

        controllers: list[JoystickType] = []

        # search for controllers
        for i in range(pygame.joystick.get_count()):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()

            controllers.append(joystick)

        for i in controllers:
            if i.get_name() == name:
                self._joystick = i
                break

        if self._joystick is None:
            raise ControllerError("No controller found")

        self.gamepad_name = self._joystick.get_name()

        # get controller layout
        self.numaxes = self._joystick.get_numaxes()
        self.numbuttons = self._joystick.get_numbuttons()
        self.numhats = self._joystick.get_numhats()

    def update(self) -> dict[str, Any]:
        if self._joystick is None:
            raise ControllerError("No controller found.")

        state = {}

        state["axes"] = [self._joystick.get_axis(i) for i in range(self.numaxes)]
        state["buttons"] = [
            self._joystick.get_button(i) for i in range(self.numbuttons)
        ]
        hats = [self._joystick.get_hat(i) for i in range(self.numhats)]
        state["dpad"] = [item for pair in hats for item in pair]

        return state


# converts logitech controller input to twist
def logitech_twist(controller_state: dict) -> Twist:
    twist = Twist()

    twist.linear.x = (
        MULTIPLIER * controller_state["axes"][4]
        if abs(controller_state["axes"][4]) > 0.15
        else 0.0
    )  # right stick vertical
    twist.linear.y = (
        MULTIPLIER * controller_state["axes"][3]
        if abs(controller_state["axes"][3]) > 0.15
        else 0.0
    )  # right stick horizontal
    # DISABLE Z MOVEMENT FOR KSED
    # twist.linear.z = MULTIPLIER * \
    #   -controller_state["axes"][1] if abs(controller_state["axes"][1]) \
    #   > 0.15 else 0.0 # left stick vertical

    twist.linear.z = 0
    twist.angular.y = MULTIPLIER * float(controller_state["dpad"][1])  # pitch
    twist.angular.x = -MULTIPLIER * float(controller_state["dpad"][0])  # roll

    yaw_r = controller_state["axes"][5]  # right trigger
    yaw_l = controller_state["axes"][2]  # left trigger
    yaw_r = (yaw_r + 1) / 2
    yaw_l = -(yaw_l + 1) / 2

    twist.angular.z = MULTIPLIER * -(yaw_r + yaw_l)
    return twist


# converts series x controller input to twist
def series_x_twist(controller_state: dict) -> Twist:
    twist = Twist()

    twist.linear.x = (
        MULTIPLIER * controller_state["axes"][3]
        if abs(controller_state["axes"][3]) > 0.15
        else 0.0
    )  # right stick vertical
    twist.linear.y = (
        MULTIPLIER * controller_state["axes"][2]
        if abs(controller_state["axes"][2]) > 0.15
        else 0.0
    )  # right stick horizontal
    twist.linear.z = (
        MULTIPLIER * -controller_state["axes"][1]
        if abs(controller_state["axes"][1]) > 0.15
        else 0.0
    )  # left stick
    twist.angular.y = MULTIPLIER * float(controller_state["dpad"][1])  # pitch
    twist.angular.x = MULTIPLIER * -float(controller_state["dpad"][0])  # roll

    yaw_r = controller_state["axes"][4]  # right trigger
    yaw_l = controller_state["axes"][5]  # left trigger
    yaw_r = (yaw_r + 1) / 2
    yaw_l = -(yaw_l + 1) / 2
    twist.angular.z = MULTIPLIER * -(yaw_r + yaw_l)

    return twist


class KsedTeleop(Node):
    def __init__(self) -> None:
        super().__init__("raw_controller")

        self._kid_enabled = False
        self._global_state = None

        pygame.init()

        self._font = pygame.font.SysFont("JetBrainsMono Nerd Font Bold", 60)
        self._display = pygame.display.set_mode((800, 400))

        # publishers
        self._cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel/teleop", 10)

        state_quality = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # subscribers
        self._state_subscription = self.create_subscription(
            State, "/state", self._state_cb, state_quality
        )

        # clients
        self._state_client = self.create_client(ChangeState, "change_state")

        # controllers
        self._parent_controller = RawController("Xbox Series X Controller")
        self._kid_logitech = None
        self._kid_joystick = None

        self._kid_logitech = RawController("Logitech Gamepad F310")

        self.timer = self.create_timer(0.05, self._update_controller)  # 20 Hz

    def _state_cb(self, msg: State) -> None:
        self._global_state = msg.state

    def _update_controller(self) -> None:
        pygame.event.pump()

        parent_state = self._parent_controller.update()

        # MENU button to enable teleop
        if parent_state["buttons"] is not None and parent_state["buttons"][11] == 1:
            msg = State()
            msg.state = State.TELEOP
            req = ChangeState.Request()
            req.new_state = msg
            self._state_client.call_async(req)

        # XBOX button to disable
        if parent_state["buttons"] is not None and parent_state["buttons"][12] == 1:
            self._kid_enabled = False
            msg = State()
            msg.state = State.DISABLED
            req = ChangeState.Request()
            req.new_state = msg
            self._state_client.call_async(req)

        # Y to enable kid control
        if parent_state["buttons"] is not None and parent_state["buttons"][4] == 1:
            self._kid_enabled = True

        # X to disable kid control
        if parent_state["buttons"] is not None and parent_state["buttons"][3] == 1:
            self._kid_enabled = False

        logi_state = None
        if self._kid_logitech is not None:
            logi_state = self._kid_logitech.update()

        self._publish_twist(parent_state, logi_state)

        self._update_display()

    def _publish_twist(self, parent: dict, kid_logi: dict | None = None, **_) -> None:
        parent_twist = series_x_twist(parent)
        kid_logi_twist = None
        kid_joy_twist = None
        if self._kid_enabled and kid_logi:
            kid_logi_twist = logitech_twist(kid_logi)

        if (
            parent_twist.linear.x == 0
            and parent_twist.linear.y == 0
            and parent_twist.linear.x == 0
            and parent_twist.angular.x == 0
            and parent_twist.angular.y == 0
            and parent_twist.angular.x == 0
        ):
            if kid_logi_twist is not None:
                self._cmd_vel_publisher.publish(kid_logi_twist)
            elif kid_joy_twist is not None:
                self._cmd_vel_publisher.publish(kid_joy_twist)
            else:
                self._cmd_vel_publisher.publish(parent_twist)
        else:
            self._cmd_vel_publisher.publish(parent_twist)

    def _update_display(self) -> None:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    raise SystemExit
                if event.key == pygame.K_SPACE:
                    self._kid_enabled = False
                    msg = State()
                    msg.state = State.DISABLED
                    req = ChangeState.Request()
                    req.new_state = msg
                    self._state_client.call_async(req)
                elif event.key == pygame.K_ESCAPE:
                    msg = State()
                    msg.state = State.TELEOP
                    req = ChangeState.Request()
                    req.new_state = msg
                    self._state_client.call_async(req)
                elif event.key == pygame.K_d:
                    self._kid_enabled = False
                elif event.key == pygame.K_s:
                    self._kid_enabled = True

        self._display.fill((0, 0, 0))

        # kid controller state
        if self._kid_enabled:
            pygame.draw.rect(self._display, (0, 255, 0), (0, 0, 400, 400))
        else:
            pygame.draw.rect(self._display, (255, 0, 0), (0, 0, 400, 400))

        enabled_str = "Kid Enabled" if self._kid_enabled else "Kid Disabled"
        kid_text = self._font.render(enabled_str, True, (0, 0, 0))
        self._display.blit(
            kid_text, (200 - kid_text.get_width() / 2, 200 - kid_text.get_height() / 2)
        )

        # global state
        state_str = ""
        state_color = (0, 0, 0)
        if self._global_state is None:
            state_str = "STARTUP"
            state_color = COLOR_STARTUP
        elif self._global_state == State.FAULT:
            state_str = "FAULT"
            state_color = COLOR_FAULT
        elif self._global_state == State.DISABLED:
            state_str = "DISABLED"
            state_color = COLOR_DISABLED
        elif self._global_state == State.TELEOP:
            state_str = "TELEOP"
            state_color = COLOR_TELEOP

        pygame.draw.rect(self._display, state_color, (400, 0, 400, 400))

        state_text = self._font.render(state_str, True, (0, 0, 0))
        self._display.blit(
            state_text,
            (600 - state_text.get_width() / 2, 200 - state_text.get_height() / 2),
        )

        # update display
        pygame.display.flip()


def main(args=None):
    rclpy.init(args=args)
    node = KsedTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
