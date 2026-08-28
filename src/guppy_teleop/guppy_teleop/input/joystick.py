# input device meant for Logitech Logitech Extreme 3D
# fair warning for this guy, has pretty bad drift on the twist meaning it will hold lock so, set low priority to override it

# from guppy_teleop.input.joystick import find_joysticks
from collections.abc import Callable  # , TYPE_CHECKING

from evdev import ecodes
from geometry_msgs.msg import Twist

from guppy_teleop.input.controller import Controller
from guppy_teleop.util.code_map import CodeMap
from guppy_teleop.util.device_mode import DeviceMode
from guppy_teleop.util.device_priority import DevicePriority

# if TYPE_CHECKING:
#    from guppy_teleop.input_handler import InputHandler


class Joystick(Controller):
    COMMAND_KEYS = [ecodes.BTN_THUMB]

    BLACKLISTED_DEVICES = []

    DEADZONE = [0.145, 0.90]

    LINEAR_MULTIPLIER = [4.0, 4.0, 2.0]
    ANGULAR_MULTIPLIER = [1.0, 1.0, 1.0]

    # default values for a Logitech Gamepad F310
    CODE_MAP = {
        "joystick_x": ecodes.ABS_X,  # type 03, from 0 to 1023
        "joystick_y": ecodes.ABS_Y,  # type 03, from 0 to 1023
        "joystick_twist": ecodes.ABS_RZ,  # type 03, from 0 to 255
        "throttle": ecodes.ABS_THROTTLE,  # type 03, 0 to 255
        "thumb_stick_x": ecodes.ABS_HAT0X,  # type 03, from -1 to 1
        "thumb_stick_y": ecodes.ABS_HAT0Y,  # type 03, from -1 to 1
        "thumb": ecodes.BTN_THUMB,  # type 01, 0 or 1
        "trigger": ecodes.BTN_TRIGGER,  # type 01, 0 or 1
        "3": ecodes.BTN_THUMB2,  # type 01, 0 or 1
        "4": ecodes.BTN_TOP,  # type 01, 0 or 1
        "5": ecodes.BTN_TOP2,  # type 01, 0 or 1
        "6": ecodes.BTN_PINKIE,  # type 01, 0 or 1
        "7": ecodes.BTN_BASE,  # type 01, 0 or 1
        "8": ecodes.BTN_BASE2,  # type 01, 0 or 1
        "9": ecodes.BTN_BASE3,  # type 01, 0 or 1
        "10": ecodes.BTN_BASE4,  # type 01, 0 or 1
        "11": ecodes.BTN_BASE5,  # type 01, 0 or 1
        "12": ecodes.BTN_BASE6,  # type 01, 0 or 1
    }

    @staticmethod
    def normalize_int10(num: int) -> float:
        return (num - 512) / 512  # can never reach 1.0

    @staticmethod
    def normalize_int8(num: int) -> float:
        return (num - 128) / 128  # can never reach 1.0

    NORMALIZE_MAP = {
        "joystick_x": normalize_int10,
        "joystick_y": normalize_int10,
        "joystick_twist": normalize_int8,
        "throttle": Controller.normalize_uint8,
    }

    _internal_code_map = CodeMap(CODE_MAP)

    def __init__(
        self,
        handler,
        path: str,
        mode=DeviceMode.DISABLED,
        name=None,
        priority=DevicePriority.LOWEST,
    ):
        super().__init__(handler, path, mode, name, priority)

        self.COMMAND_MAP: dict[Callable, list[int]] = {
            lambda: self.handler.push_state("FAULT"): [
                ecodes.BTN_THUMB,
                ecodes.BTN_TRIGGER,
            ],
            lambda: self.handler.push_state("TELEOP"): [
                ecodes.BTN_THUMB,
                ecodes.BTN_BASE5,
            ],
            lambda: self.handler.push_state("DISABLED"): [
                ecodes.BTN_THUMB,
                ecodes.BTN_BASE6,
            ],
            lambda: self.handler.lock_priority(self.priority): [
                ecodes.BTN_THUMB,
                ecodes.BTN_BASE3,
            ],
            lambda: self.handler.lock_priority(None): [
                ecodes.BTN_THUMB,
                ecodes.BTN_BASE4,
            ],
            lambda: self._cycle_mode(DeviceMode.DISABLED): [
                ecodes.BTN_THUMB,
                ecodes.BTN_BASE2,
            ],
        }

        self._state = {
            "joystick_x": 0.0,  # normalized -1 -> 1
            "joystick_y": 0.0,  # normalized -1 -> 1
            "joystick_twist": 0.0,  # normalized -1 -> 1
            "throttle": 0.0,  # normalized 0 -> 1
            "thumb_stick_x": 0,  # from -1 -> 1
            "thumb_stick_y": 0,  # from -1 -> 1
            "thumb": False,  # True or False
            "trigger": False,  # True or False
            "3": False,  # True or False
            "4": False,  # True or False
            "5": False,  # True or False
            "6": False,  # True or False
            "7": False,  # True or False
            "8": False,  # True or False
            "9": False,  # True or False
            "10": False,  # True or False
            "11": False,  # True or False
            "12": False,  # True or False
        }

    def transform(self, snapshot: dict) -> Twist:
        twist = Twist()

        throttle = 1.0 - snapshot["throttle"]

        twist.linear.x = self.stick_deadzone(
            float(snapshot["joystick_y"]), self.LINEAR_MULTIPLIER[0] * throttle
        )
        twist.linear.y = -self.stick_deadzone(
            float(snapshot["joystick_x"]), self.LINEAR_MULTIPLIER[1] * throttle
        )
        twist.linear.z = (
            -float(snapshot["thumb_stick_y"]) *
            self.LINEAR_MULTIPLIER[2] * throttle
        )

        twist.angular.x = (
            float(snapshot["thumb_stick_x"]) *
            self.ANGULAR_MULTIPLIER[0] * throttle
        )
        twist.angular.y = (
            (float(snapshot["5"]) - float(snapshot["6"]))
            * self.ANGULAR_MULTIPLIER[1]
            * throttle
        )
        twist.angular.z = -self.stick_deadzone(
            float(snapshot["joystick_twist"]
                  ), self.ANGULAR_MULTIPLIER[2] * throttle
        )

        return twist

    def package(self, snapshot: dict) -> dict:
        return {  # TODO THESE NEED TUNING
            "name": self.name,
            "format": "joystick",
            "input": {},
        }


# import asyncio

# async def start():
#     devices = find_joysticks(None)

#     tasks = [asyncio.create_task(device.start()) for device in devices]

#     if len(tasks) == 0:
#         print("no devices bruh")
#         return

#     done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_EXCEPTION)

#     for task in pending:
#         task.cancel()

#     for task in done:
#         if task.exception():
#             raise task.exception()


# def main():
#     asyncio.run(start())

if __name__ == "__main__":
    from guppy_teleop.util.find_devices import find_joysticks

    print(find_joysticks(None))
