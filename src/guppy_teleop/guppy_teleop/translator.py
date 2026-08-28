import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray, String


class Translator(Node):
    def __init__(self):
        super().__init__("translator")

        self.dpad_subscriber = self.create_subscription(
            Int32MultiArray, "dpad", self.dpad_callback, 10
        )
        self.axes_subscriber = self.create_subscription(
            Float32MultiArray, "axes", self.axes_callback, 10
        )
        self.button_subscriber = self.create_subscription(
            Int32MultiArray, "buttons", self.button_callback, 10
        )
        self.name_subscriber = self.create_subscription(
            String, "gamepad_name", self.name_callback, 10
        )

        self.publisher = self.create_publisher(Twist, "/cmd_vel/teleop", 10)

        self.controller_state = {"dpad": None, "axes": None, "buttons": None}
        self.controller_name = None

    def dpad_callback(self, msg):
        self.get_logger().debug(f"Received dpad message: {msg.data}")
        self.controller_state["dpad"] = msg.data
        self.update_controller_state()

    def axes_callback(self, msg):
        self.get_logger().debug(f"Received axes message: {msg.data}")
        self.controller_state["axes"] = msg.data
        self.update_controller_state()

    def button_callback(self, msg):
        self.get_logger().debug(f"Received button message: {msg.data}")
        self.controller_state["buttons"] = msg.data
        self.update_controller_state()

    def name_callback(self, msg):
        self.controller_name = msg.data

    def update_controller_state(self):
        if (
            self.controller_state["dpad"] is not None
            and self.controller_state["axes"] is not None
            and self.controller_state["buttons"] is not None
            and self.controller_name is not None
        ):
            self.publish_twist()
            self.controller_state = {
                "dpad": None, "axes": None, "buttons": None}

    def publish_twist(self):
        if (
            self.controller_state["dpad"] is not None
            and self.controller_state["axes"] is not None
            and self.controller_state["buttons"] is not None
            and self.controller_name is not None
        ):
            twist = None
            if self.controller_name == "Logitech Gamepad F310":
                twist = logitech_twist(self.controller_state)
            elif self.controller_name == "Xbox Series X Controller":
                twist = series_x_twist(self.controller_state)

            if twist is not None:
                self.publisher.publish(twist)


def logitech_twist(controller_state):
    twist = Twist()

    twist.linear.x = (
        -controller_state["axes"][4] if abs(controller_state["axes"][4]) > 0.05 else 0.0
    )  # right stick vertical
    twist.linear.y = (
        -controller_state["axes"][3] if abs(controller_state["axes"][3]) > 0.05 else 0.0
    )  # right stick horizontal
    twist.linear.z = (
        -controller_state["axes"][1] if abs(controller_state["axes"][1]) > 0.05 else 0.0
    )  # left stick vertical
    twist.angular.y = -float(controller_state["dpad"][1])  # pitch
    twist.angular.x = -float(controller_state["dpad"][0])  # roll
    yaw_r = controller_state["axes"][5]  # right trigger
    yaw_l = controller_state["axes"][2]  # left trigger
    yaw_r = (yaw_r + 1) / 2
    yaw_l = (yaw_l + 1) / 2 * (-1)
    twist.angular.z = yaw_r + yaw_l

    return twist


def series_x_twist(controller_state):
    twist = Twist()
    twist.linear.x = controller_state["axes"][4]  # right stick vertical
    twist.linear.y = controller_state["axes"][3]  # right stick horizontal
    twist.linear.z = -controller_state["axes"][1]  # left stick
    twist.angular.y = float(controller_state["dpad"][1])  # pitch
    twist.angular.x = -float(controller_state["dpad"][0])  # roll
    yaw_r = controller_state["axes"][5]  # right trigger
    yaw_l = controller_state["axes"][2]  # left trigger
    yaw_r = (yaw_r + 1) / 2
    yaw_l = (yaw_l + 1) / 2 * (-1)
    twist.angular.z = -(yaw_r + yaw_l)
    return twist


def main(Args=None):
    rclpy.init(args=Args)

    translator = Translator()

    rclpy.spin(translator)

    translator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
