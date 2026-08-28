import asyncio
import threading
import time
from collections.abc import Callable
from enum import Enum
from queue import Queue

import rclpy
from geometry_msgs.msg import Twist
from guppy_msgs.srv._change_state import ChangeState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from guppy_msgs.msg import State
from guppy_teleop.input.input_device import InputDevice
from guppy_teleop.util.device_mode import DeviceMode
from guppy_teleop.util.device_priority import DevicePriority
from guppy_teleop.util.find_devices import find_controllers, find_keyboards


class ValidState(Enum):  # TODO replace with State constants directly
    STARTUP = 0
    HOLDING = 1
    NAV = 2
    TASK = 3
    TELEOP = 4
    DISABLED = 5
    FAULT = 6


class InputHandler(Node):
    TIMEOUT = 0.5  # seconds before input device gives up focus

    def __init__(self, update_widget_callback: Callable | None = None):
        super().__init__("teleop_input")

        self._update_widget_callback = update_widget_callback

        self.queue = Queue()

        self.input_thread: threading.Thread = None

        self.input_devices: list[InputDevice] = []

        self._focus: InputDevice = None

        self._priority_lock = None

        quality = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.publisher = self.create_publisher(
            Twist, "/cmd_vel/teleop", quality)

        self._client = self.create_client(ChangeState, "change_state")
        self.req = None
        if not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("change_state service not available")
        else:
            self.req = ChangeState.Request()

        self.timer = self.create_timer(0.05, self._watchdog)

    def thread_start(self):
        self.input_thread = threading.Thread(
            target=lambda: asyncio.run(self.start_devices()), daemon=True
        )
        self.input_thread.start()

    async def start_devices(self):
        if not self.input_devices:
            return

        try:
            async with asyncio.TaskGroup() as group:
                for device in self.input_devices:
                    group.create_task(device.start())
        except* Exception as exception_group:
            for exception in exception_group.exceptions:
                print(f"device input crashed\n{exception}")
        finally:
            print("stopped full")

    def on_device_event(
        self, device: InputDevice, snapshot: dict
    ):  # passes snapshot of state to prevent race conditions of reading the active state
        if not device.active:
            # see thread-safe issue on _watchdog(), has the potential to eat inputs,
            # possibly safe to remove condition as by recieving a event means the device
            # is "active" and can have last_time set to the current time?
            return
        if self._priority_lock is not None and device.priority > self._priority_lock:
            return
        elif self._focus is None or device.priority < self._focus.priority:
            self._focus = device
        elif self._focus != device:
            return

        if self._update_widget_callback:
            self._update_widget_callback(self._focus.package(snapshot))

        self.publisher.publish(self._focus.transform(snapshot))

    # this guy is NOT thread-safe and can cause a race-condition while attempting to mark the device inactive HOWEVER
    # that's okkkkkk. a zero twist message will STILL be published (preventing hanging) and focus will be lost essentially
    # eating a single input which kinda sucks but might happen. just call it a skill issue.
    def _watchdog(self):
        if self._focus:
            if self._focus.heartbeat():
                return

            dt = time.time() - self._focus.last_active
            if dt > self.TIMEOUT:
                self._focus._mark_inactive()
                self._focus = None

                self.publisher.publish(Twist())

    def add_device(self, *devices: InputDevice):
        for device in devices:
            device.handler = self

            self.input_devices.append(device)

    def lock_priority(self, priority: DevicePriority):
        self._priority_lock = priority
        print(f"priority locked to '{priority}'!")

    def push_state(self, new_state: str):
        message = State()
        if (valid_state := getattr(ValidState, new_state, None)) is None:
            return False

        if self.req is None:
            return False

        message.state = valid_state.value
        self.req.new_state = message

        print(f"state change to '{new_state}'!")

        future = self._client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success


def main(args=None):
    rclpy.init()

    handler = InputHandler(None)

    handler.add_device(
        *find_controllers(handler, DeviceMode.INPUT, DevicePriority.MEDIUM, True)
    )
    handler.add_device(
        *find_keyboards(handler, DeviceMode.COMMAND, DevicePriority.MEDIUM, True)
    )

    handler.thread_start()

    try:
        rclpy.spin(handler)
    finally:
        handler.destroy_node()
        rclpy.shutdown()


def controller(args=None):
    rclpy.init()

    handler = InputHandler(None)

    handler.add_device(
        *find_controllers(handler, DeviceMode.INPUT, DevicePriority.MEDIUM, True)
    )
    handler.add_device(
        *find_keyboards(handler, DeviceMode.COMMAND, DevicePriority.MEDIUM, True)
    )

    handler.thread_start()

    try:
        rclpy.spin(handler)
    finally:
        handler.destroy_node()
        rclpy.shutdown()


def keyboard(args=None):
    rclpy.init()

    handler = InputHandler(None)

    handler.add_device(
        *find_controllers(handler, DeviceMode.INPUT, DevicePriority.MEDIUM, True)
    )
    handler.add_device(
        *find_keyboards(handler, DeviceMode.COMMAND, DevicePriority.MEDIUM, True)
    )

    handler.thread_start()

    try:
        rclpy.spin(handler)
    finally:
        handler.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
