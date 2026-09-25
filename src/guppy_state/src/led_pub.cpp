#include "guppy_msgs/msg/state.hpp"
#include "guppy_msgs/srv/send_can.hpp"
#include "guppy_util/quality.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"

#include <memory>

using namespace std::chrono_literals;

class LEDStatePublisher : public rclcpp::Node {
private:
    uint8_t                                                       current_state = 0;
    std::shared_ptr<rclcpp::Subscription<guppy_msgs::msg::State>> subscription_;
    std::shared_ptr<rclcpp::Client<guppy_msgs::srv::SendCan>>     client_;
    std::shared_ptr<rclcpp::TimerBase>                            timer;
public:
    LEDStatePublisher() : Node("led_pub") {
        auto topic_callback = [this](std::unique_ptr<guppy_msgs::msg::State> msg) -> void {
            current_state = msg->state;
        };
        subscription_ = this->create_subscription<guppy_msgs::msg::State>(
            "state", quality::reliable_profile, topic_callback
        );
        client_ = this->create_client<guppy_msgs::srv::SendCan>("can_tx");
        timer   = this->create_wall_timer(100ms, [this]() {
            auto request  = std::make_shared<guppy_msgs::srv::SendCan::Request>();
            request->id   = 0x201;
            request->data = {current_state};
            client_->async_send_request(request);
        });
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    const auto led_publisher = std::make_shared<LEDStatePublisher>();
    rclcpp::spin(led_publisher);
    rclcpp::shutdown();
    return 0;
}
