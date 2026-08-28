import sys
import threading
from pathlib import Path

import rclpy
from PySide6 import QtCore
from PySide6.QtCore import QObject
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine
from rclpy.executors import MultiThreadedExecutor

from guppy_teleop.frontend.toast_service import ToastService
from guppy_teleop.frontend.widgets.input_widget import InputWidget
from guppy_teleop.frontend.widgets.param_widget import ParameterWidget
from guppy_teleop.frontend.widgets.state_widget import StateWidget
from guppy_teleop.frontend.widgets.widget import Widget
from guppy_teleop.frontend.workspace_manager import WorkspaceManager


def main(args=None):
    print(QtCore.__version__)

    rclpy.init(args=args)

    # start qt
    app = QGuiApplication(sys.argv)
    engine = QQmlApplicationEngine()

    # start workspace manager
    manager = WorkspaceManager()
    engine.rootContext().setContextProperty("workspaceManager", manager)

    widgets: list[Widget] = []

    # widgets here

    widgets.append(StateWidget())
    widgets.append(ParameterWidget())
    widgets.append(InputWidget())

    # widgets end

    # widget setup
    executor = MultiThreadedExecutor()
    for widget in widgets:
        engine.rootContext().setContextProperty(widget.qml_name, widget)
        executor.add_node(widget)

    # initialize qt connections and modules
    engine.addImportPath(str(Path(__file__).parent))
    engine.loadFromModule("ui", "Main")

    if not engine.rootObjects():
        print("something went pretty wrong")
        sys.exit(-1)

    root = None
    for object in engine.rootObjects():
        if object.objectName() == "window":
            root = object
            break

    # setup toast service
    toast_service = ToastService()
    toast_service.toastManager = root.findChild(QObject, "toastManager")
    executor.add_node(toast_service)

    # start ros in seperate thread
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    try:
        app.exec()
    finally:
        app.quit()

        executor.shutdown()
        ros_thread.join(timeout=2.0)

        for widget in widgets:
            executor.remove_node(widget)
            widget.destroy_node()

        rclpy.shutdown()
        print("app exited")

    del engine


if __name__ == "__main__":
    main()
