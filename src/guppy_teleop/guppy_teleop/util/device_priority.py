from enum import Enum


class DevicePriority(Enum):
    HIGHEST = (0,)
    MEDIUM = (1,)
    LOWEST = (2,)
    CHILD = 3

    def __lt__(self, other):
        if self.__class__ is other.__class__:
            return self.value < other.value

        return NotImplemented

    def __gt__(self, other):
        if self.__class__ is other.__class__:
            return self.value > other.value

        return NotImplemented
