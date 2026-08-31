from enum import Enum


class DeviceMode(Enum):
    DISABLED = (0,)
    COMMAND = (1,)
    INPUT = 2

    def next(self):
        modes = list(type(self))
        index = modes.index(self)
        return modes[(index + 1) % len(modes)]
