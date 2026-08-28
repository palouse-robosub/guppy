from collections.abc import Callable  # , TYPE_CHECKING

from evdev import InputDevice as EvdevDevice  # , categorize
from evdev import KeyEvent, ecodes
from geometry_msgs.msg import Twist

from guppy_teleop.input.input_device import InputDevice
from guppy_teleop.util.code_map import CodeMap
from guppy_teleop.util.device_mode import DeviceMode
from guppy_teleop.util.device_priority import DevicePriority

# if TYPE_CHECKING:
#    from guppy_teleop.input_handler import InputHandler


class Keyboard(InputDevice):
    COMMAND_KEYS = [
        ecodes.KEY_LEFTCTRL,
        ecodes.KEY_RIGHTCTRL,
        ecodes.KEY_LEFTALT,
        ecodes.KEY_RIGHTALT,
    ]

    BLACKLISTED_DEVICES = ["Logitech G300s Optical Gaming Mouse Keyboard"]

    LINEAR_MULTIPLIER = [8.0, 8.0, 8.0]
    ANGULAR_MULTIPLIER = [8.0, 8.0, 8.0]

    VALID_TYPES = [ecodes.EV_ABS, ecodes.EV_KEY]

    CODE_MAP = {
        "q": ecodes.KEY_Q,
        "w": ecodes.KEY_W,
        "e": ecodes.KEY_E,
        "a": ecodes.KEY_A,
        "s": ecodes.KEY_S,
        "d": ecodes.KEY_D,
        "shift": ecodes.KEY_LEFTSHIFT,
        "space": ecodes.KEY_SPACE,
        "up": ecodes.KEY_UP,
        "down": ecodes.KEY_DOWN,
        "left": ecodes.KEY_LEFT,
        "right": ecodes.KEY_RIGHT,
    }

    _internal_code_map = CodeMap(CODE_MAP)

    def __init__(
        self,
        handler,
        path: str,
        mode=DeviceMode.DISABLED,
        name=None,
        priority=DevicePriority.MEDIUM,
    ):
        self._device = EvdevDevice(path)

        super().__init__(mode, name if name else self._device.name, priority)

        self.handler = handler

        self.COMMAND_MAP: dict[Callable, list[int]] = {
            lambda: self.handler.push_state("FAULT"): [
                ecodes.KEY_LEFTCTRL,
                ecodes.KEY_SPACE,
            ],
            lambda: self.handler.push_state("TELEOP"): [
                ecodes.KEY_LEFTCTRL,
                ecodes.KEY_T,
            ],
            lambda: self.handler.push_state("DISABLED"): [
                ecodes.KEY_LEFTCTRL,
                ecodes.KEY_D,
            ],
            lambda: self.handler.lock_priority(self.priority): [
                ecodes.KEY_LEFTCTRL,
                ecodes.KEY_L,
            ],
            lambda: self.handler.lock_priority(None): [
                ecodes.KEY_LEFTCTRL,
                ecodes.KEY_K,
            ],
            lambda: self._cycle_mode(DeviceMode.DISABLED): [
                ecodes.KEY_LEFTCTRL,
                ecodes.KEY_E,
            ],
        }

        self._state = {
            "q": KeyEvent.key_up,
            "w": KeyEvent.key_up,
            "e": KeyEvent.key_up,
            "a": KeyEvent.key_up,
            "s": KeyEvent.key_up,
            "d": KeyEvent.key_up,
            "shift": KeyEvent.key_up,
            "space": KeyEvent.key_up,
            "up": KeyEvent.key_up,
            "down": KeyEvent.key_up,
            "left": KeyEvent.key_up,
            "right": KeyEvent.key_up,
        }

        print(f"'{self.name}' initialized as {self.mode.name} as {self.priority.name}!")

    async def start(self):
        async for event in self._device.async_read_loop():
            self._process(event)

    def _process(self, event):
        if self.mode == DeviceMode.DISABLED:
            return

        if event.type == ecodes.EV_KEY:
            if (
                not self._command_mode
                and event.code in self.COMMAND_KEYS
                and event.value == KeyEvent.key_down
            ):
                self._command_mode = True

            if self._command_mode and event.value == KeyEvent.key_down:
                active_keys = self._device.active_keys()

                if any(key in active_keys for key in self.COMMAND_KEYS):
                    for command, keys in self.COMMAND_MAP.items():
                        if event.code not in keys:
                            continue

                        if all(key in active_keys for key in keys):
                            command()
                            # remember to use try catch in your commands
                            # or it will crash with no error!

                    return
                else:
                    self._command_mode = False

                    if event.code in self.COMMAND_KEYS:
                        return

            if (code_name := self._internal_code_map[event.code]) is not None:
                if self.mode != DeviceMode.INPUT:
                    return

                self._state[code_name] = event.value

                self._mark_active()
                if self.handler:
                    self.handler.on_device_event(self, self._state.copy())

            # print(categorize(event))
            # print(f"{event.code}, {event.type}, {event.value}")

    def transform(self, snapshot: dict) -> Twist:
        twist = Twist()

        q = snapshot["q"] != KeyEvent.key_up
        w = snapshot["w"] != KeyEvent.key_up
        e = snapshot["e"] != KeyEvent.key_up
        a = snapshot["a"] != KeyEvent.key_up
        s = snapshot["s"] != KeyEvent.key_up
        d = snapshot["d"] != KeyEvent.key_up
        shift = snapshot["shift"] != KeyEvent.key_up
        space = snapshot["space"] != KeyEvent.key_up
        right = snapshot["right"] != KeyEvent.key_up
        left = snapshot["left"] != KeyEvent.key_up
        up = snapshot["up"] != KeyEvent.key_up
        down = snapshot["down"] != KeyEvent.key_up

        twist.linear.x = float(w - s) * self.LINEAR_MULTIPLIER[0]
        twist.linear.y = -float(d - a) * self.LINEAR_MULTIPLIER[1]
        twist.linear.z = float(space - shift) * self.LINEAR_MULTIPLIER[2]

        twist.angular.x = float(up - down) * self.ANGULAR_MULTIPLIER[1]
        twist.angular.y = float(right - left) * self.ANGULAR_MULTIPLIER[0]
        twist.angular.z = -float(e - q) * self.ANGULAR_MULTIPLIER[2]

        return twist

    def package(self, snapshot: dict) -> dict:
        return {"name": self.name, "format": "keyboard", "input": snapshot}

    def heartbeat(self):
        if self._command_mode or self.mode != DeviceMode.INPUT:
            return False

        for key in self._device.active_keys():
            if (code_name := self._internal_code_map[key]) is None:
                continue
            if self._state[code_name]:
                return True

        return False


if __name__ == "__main__":
    from guppy_teleop.util.find_devices import find_keyboards

    print(find_keyboards(None))
