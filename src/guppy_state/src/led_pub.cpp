#include <memory>
#include <vector>

#include "guppy_msgs/msg/state.hpp"
#include "guppy_msgs/srv/send_can.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class LEDStatePublisher : public rclcpp::Node {
 public:
  LEDStatePublisher() : Node("led_pub") {
    auto topic_callback = [this](guppy_msgs::msg::State::UniquePtr msg) -> void { current_state = msg->state; };

    subscription_ = this->create_subscription<guppy_msgs::msg::State>("state", 10, topic_callback);
    client_ = this->create_client<guppy_msgs::srv::SendCan>("can_tx");

    timer = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
      auto request = std::make_shared<guppy_msgs::srv::SendCan::Request>();
      request->id = 0x201;
      request->data = {current_state};
      client_->async_send_request(request);
    });
  }

 private:
  uint8_t current_state = 0;
  rclcpp::Subscription<guppy_msgs::msg::State>::SharedPtr subscription_;
  rclcpp::Client<guppy_msgs::srv::SendCan>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LEDStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
