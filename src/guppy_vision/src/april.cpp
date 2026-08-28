#include <memory>
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class AprilRepub : public rclcpp::Node {
 public:
  AprilRepub() : Node("aprilrepub") {
    auto qos = rclcpp::SensorDataQoS();

    // free-function API: takes Node*, no shared_from_this() needed
    image_pub_ = image_transport::create_publisher(this, "/cam/anno", qos.get_rmw_qos_profile());

    sub_ = image_transport::create_subscription(this, "/cam/test_annotated", std::bind(&AprilRepub::cb, this, std::placeholders::_1),
                                                "raw",  // transport hint
                                                qos.get_rmw_qos_profile());
  }

 private:
  // ConstSharedPtr, not a bare reference
  void cb(const sensor_msgs::msg::Image::ConstSharedPtr& msg) { image_pub_.publish(*msg); }

  image_transport::Publisher image_pub_;
  image_transport::Subscriber sub_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AprilRepub>());
  rclcpp::shutdown();
  return 0;
}