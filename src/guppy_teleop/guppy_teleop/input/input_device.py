from abc import ABC, abstractmethod
from time import time

from geometry_msgs.msg import Twist

from guppy_teleop.util.device_mode import DeviceMode
from guppy_teleop.util.device_priority import DevicePriority


# TODO replace enabled with a mode: DISABLED, COMMAND, INPUT so that a device can only send commands (ie. the keyboard so it doesn't send things while typing)
class InputDevice(ABC):
    def __init__(
        self,
        mode: DeviceMode = DeviceMode.DISABLED,
        name: str = "Unkown Input Device",
        priority: DevicePriority = DevicePriority.MEDIUM,
    ):
        # bad design but whatever, makes it convienient to call commands without passing in lots of callbacks
        self.handler = None

        self.mode = mode
        self.name = name
        self.priority = priority

        self.active = False
        self.last_active = 0.0

        self._command_mode = False

        self._state: dict = None

    def _mark_active(self):
        self.active = True
        self.last_active = time()

    def _mark_inactive(self):
        self.active = False

    def _cycle_mode(
        self, *skip: DeviceMode
    ):  # TODO devices need better control rather than just cycling, or the method of cycling is still available while disabled
        if set(DeviceMode) == set(skip):
            raise AssertionError("cannot skip all items in cycle!")

        mode = self.mode.next()
        while mode in skip:
            mode = mode.next()

        self.mode = mode

    @abstractmethod
    async def start(self): ...

    @abstractmethod
    def transform(self, snapshot: dict) -> Twist: ...

    @abstractmethod
    def package(self, snapshot: dict) -> dict: ...

    @abstractmethod
    def heartbeat(self) -> bool: ...
