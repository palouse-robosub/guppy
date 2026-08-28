import rclpy
from PySide6.QtCore import Property, Signal, Slot
from rcl_interfaces.msg import ParameterEvent, SetParametersResult
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.parameter_event_handler import ParameterEventHandler
from rclpy.task import Future

from guppy_teleop.frontend.widgets.widget import Widget


class ParameterWidget(Node, Widget):
    parametersChanged = Signal()

    @property
    def qml_name(self) -> str:
        return "parameterWidget"

    def __init__(self, parent=None):
        Node.__init__(self, "parameter_widget")
        Widget.__init__(self, parent)
        self._params = {}

        self.client = AsyncParameterClient(self, "control_chassis")

        self._load_parameters()

        self.handler = ParameterEventHandler(self)
        self.event_callback_handle = self.handler.add_parameter_event_callback(
            callback=self._on_param_change
        )

    def _load_parameters(self):
        list_client = self.create_client(
            ListParameters, "control_chassis/list_parameters"
        )
        get_client = self.create_client(
            GetParameters, "control_chassis/get_parameters")

        if not list_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                "parameter widget couldn't list control parameters!"
            )
            return

        if not get_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("parameter widget couldn't get control parameters!")
            return

        list_request = ListParameters.Request()
        future = list_client.call_async(list_request)
        rclpy.spin_until_future_complete(self, future)

        names = future.result().result.names

        get_request = GetParameters.Request()
        get_request.names = names
        future = get_client.call_async(get_request)
        rclpy.spin_until_future_complete(self, future)

        values = future.result().values

        for name, value in zip(names, values, strict=False):
            self._params[name] = rclpy.parameter.parameter_value_to_python(
                value)

    def _on_param_change(self, event: ParameterEvent):
        for param in event.changed_parameters:
            self._params[param.name] = rclpy.parameter.parameter_value_to_python(
                param.value
            )

        self.parametersChanged.emit()

    @Property("QVariantMap", notify=parametersChanged)
    def parameters(self):
        return self._params

    @Slot("QVariantMap")
    def pushParameters(self, params: dict) -> bool:
        try:
            requests: Parameter = []

            for key, value in params.items():
                if not value.__eq__(self._params[key]):
                    if isinstance(value, list):
                        value = list(map(float, value))
                    requests.append(Parameter(name=key, value=value))

            if len(requests) == 0:
                return True

            print(requests)

            if not self.client.wait_for_services(timeout_sec=2.0):
                raise RuntimeError("parameter service not available")

            future: Future[SetParametersResult] = self.client.set_parameters(
                requests)
            rclpy.spin_until_future_complete(self, future)

            response: SetParameters.Response = future.result()

            results: list[SetParametersResult] = response.results

            for result in results:
                if not result.successful:
                    raise (Exception(result.reason))

        except Exception as err:
            self.get_logger().error(f"failed to update parameters: {err!s}!")
            return False

        return True
