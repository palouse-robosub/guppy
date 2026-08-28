

from evdev import KeyEvent
from PySide6.QtCore import Property, Signal

from guppy_teleop.frontend.widgets.widget import Widget
from guppy_teleop.input_handler import InputHandler
from guppy_teleop.util.device_mode import DeviceMode
from guppy_teleop.util.device_priority import DevicePriority
from guppy_teleop.util.find_devices import find_controllers, find_keyboards


class InputWidget(InputHandler, Widget):
    inputUpdated = Signal()

    @property
    def qml_name(self) -> str:
        return "inputWidget"

    def __init__(self, parent=None):
        InputHandler.__init__(self, self.update)
        Widget.__init__(self, parent)

        self.add_device(
            *find_controllers(self, DeviceMode.INPUT, DevicePriority.MEDIUM)
        )
        self.add_device(
            *find_keyboards(self, DeviceMode.COMMAND, DevicePriority.HIGHEST)
        )

        self._input_package = {
            "name": "test keyboard",
            "format": "keyboard",
            "input": {
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
            },
        }

        self.thread_start()

    @Property("QVariantMap", notify=inputUpdated)
    def inputPackage(self) -> dict:
        return self._input_package

    def update(self, input_package: dict):
        self._input_package = input_package

        self.inputUpdated.emit()
