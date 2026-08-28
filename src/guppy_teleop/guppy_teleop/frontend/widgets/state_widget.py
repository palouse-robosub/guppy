from enum import Enum

import rclpy
from guppy_msgs.srv._change_state import ChangeState
from PySide6.QtCore import Property, Signal, Slot
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from guppy_msgs.msg import State
from guppy_teleop.frontend.widgets.widget import Widget


class ValidState(Enum):  # TODO replace with State constants directly
    STARTUP = 0
    HOLDING = 1
    NAV = 2
    TASK = 3
    TELEOP = 4
    DISABLED = 5
    FAULT = 6


class StateWidget(Node, Widget):
    stateChanged = Signal()

    @property
    def qml_name(self) -> str:
        return "stateWidget"

    def __init__(self, parent=None):
        Node.__init__(self, "state_widget")
        Widget.__init__(self, parent)
        self._state: str = "no state recieved"

        self._client = self.create_client(ChangeState, "change_state")
        self.req = None
        if not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("change_state service not available")
        else:
            self.req = ChangeState.Request()

        state_quality = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(State, "state", self._on_state, state_quality)

    @Property(str, notify=stateChanged)
    def state(self) -> str:
        return self._state

    @Slot(str, result=bool)
    def pushState(self, new_state: str) -> bool:
        message = State()
        if (valid_state := getattr(ValidState, new_state, None)) is None:
            return False

        if self.req is None:
            return False

        message.state = valid_state.value
        self.req.new_state = message

        future = self._client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def _on_state(self, message: State):
        try:
            self._state = ValidState(message.state).name
        except ValueError:
            self._state = "UNKNOWN"

        self.stateChanged.emit()
