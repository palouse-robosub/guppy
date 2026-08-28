from typing import Any

import pygame
import rclpy
from pygame.joystick import JoystickType
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray, String


class ControllerModeError(Exception):
    """Raised when the Logitech Gamepad is not in XInput mode."""

    def __init__(self, msg=None):
        if msg is None:
            msg = "Please ensure the Logitech Gamepad is in XInput mode using the switch on the bottom."
        super().__init__(msg)


class RawController:
    def __init__(self) -> None:
        pygame.init()
        pygame.joystick.init()
        self._joystick = None

        controllers: list[JoystickType] = []

        # search for controllers
        for i in range(pygame.joystick.get_count()):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()

            controllers.append(joystick)

        for i in controllers:
            if i.get_name() == "Logitech Gamepad F310":
                self._joystick = i
            elif i.get_name() == "Logitech Dual Action":
                raise ControllerModeError()

        if not self._joystick:
            for i in controllers:
                if i:
                    self._joystick = i
                    break

        if self._joystick is None:
            raise ValueError("No controller found")

        self.gamepad_name = self._joystick.get_name()

        # get controller layout
        self.numaxes = self._joystick.get_numaxes()
        self.numbuttons = self._joystick.get_numbuttons()
        self.numhats = self._joystick.get_numhats()

    def update(self) -> dict[str, Any]:
        if self._joystick is None:
            raise ValueError("No controller found")
        pygame.event.pump()
        state = {}
        state["axes"] = [self._joystick.get_axis(
            i) for i in range(self.numaxes)]
        state["buttons"] = [
            self._joystick.get_button(i) for i in range(self.numbuttons)
        ]
        state["hats"] = [self._joystick.get_hat(
            i) for i in range(self.numhats)]
        state["name"] = self.gamepad_name
        return state


class RawControllerPublisher(Node):
    def __init__(self):
        super().__init__("raw_controller")
        # create publishers
        self.dpad_publisher = self.create_publisher(
            Int32MultiArray, "dpad", 10)
        self.axes_publisher = self.create_publisher(
            Float32MultiArray, "axes", 10)
        self.button_publisher = self.create_publisher(
            Int32MultiArray, "buttons", 10)
        self.name_publisher = self.create_publisher(String, "gamepad_name", 10)

        self.controller = RawController()
        self.timer = self.create_timer(0.05, self.publish_controller)  # 20 Hz

    def publish_controller(self):
        try:
            state = self.controller.update()
            dpad_msg = Int32MultiArray()
            dpad_msg.data = [item for pair in state["hats"] for item in pair]
            self.dpad_publisher.publish(dpad_msg)
            axes_msg = Float32MultiArray()
            axes_msg.data = state["axes"]
            self.axes_publisher.publish(axes_msg)
            button_msg = Int32MultiArray()
            button_msg.data = state["buttons"]
            self.button_publisher.publish(button_msg)
            name_msg = String()
            name_msg.data = state["name"]
            self.name_publisher.publish(name_msg)
            self.get_logger().debug(f"Published dpad: {dpad_msg.data!s}")
            self.get_logger().debug(f"Published axes: {axes_msg.data!s}")
            self.get_logger().debug(f"Published buttons: {button_msg.data!s}")
            self.get_logger().debug(f"Published name: {name_msg.data!s}")
        except Exception as e:
            self.get_logger().error(f"Controller read failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RawControllerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
