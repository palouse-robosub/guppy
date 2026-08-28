from abc import ABCMeta, abstractmethod

from PySide6.QtCore import QObject


# can't inherit from classes with incompatible metaclasses so this provides a skeleton for a compatible one
class ABCQObjectMeta(ABCMeta, type(QObject)):
    ...


class Widget(QObject, metaclass=ABCQObjectMeta):
    @property
    @abstractmethod
    def qml_name(self) -> str: ...
