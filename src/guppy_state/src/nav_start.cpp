#include <memory>

#include "guppy_msgs/msg/can_frame.hpp"
#include "guppy_msgs/msg/state.hpp"
#include "guppy_msgs/srv/change_state.hpp"
#include "guppy_util/quality.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/executors.hpp"

using namespace std::chrono_literals;

class NavStart : public rclcpp::Node {
private:
    std::shared_ptr<rclcpp::Subscription<guppy_msgs::msg::CanFrame>> nav_switch_sub_;
    std::shared_ptr<rclcpp::Client<guppy_msgs::srv::ChangeState>>    state_client_;
    bool                                                       was_nav_ = false;
    bool                                                       initialized_ = false;
public:
    NavStart() : Node("navstart") {
        nav_switch_sub_ = this->create_subscription<guppy_msgs::msg::CanFrame>(
            "/can/id_0x22",
            quality::reliable_profile,
            [this](guppy_msgs::msg::CanFrame msg) {
                this->switch_callback(msg);
            }
        );
        state_client_ = this->create_client<guppy_msgs::srv::ChangeState>("change_state");
    }
private:
    void switch_callback(guppy_msgs::msg::CanFrame msg) {
        int nav_on = 0;
        memcpy(&nav_on, msg.data.data(), sizeof(int));
        if (!initialized_) {
            initialized_ = true;
            was_nav_     = nav_on;
            return;
        }
        if (!nav_on && was_nav_) {
            auto request = std::make_shared<guppy_msgs::srv::ChangeState::Request>();
            guppy_msgs::msg::State state;
            state.state        = guppy_msgs::msg::State::HOLDING;
            request->new_state = state;
            state_client_->async_send_request(request);
            was_nav_ = false;
        } else if (nav_on && !was_nav_) {
            auto request = std::make_shared<guppy_msgs::srv::ChangeState::Request>();
            guppy_msgs::msg::State state;
            state.state        = guppy_msgs::msg::State::NAV;
            request->new_state = state;
            state_client_->async_send_request(request);
            was_nav_ = true;
            // std::thread t([](){
            //     system("/home/robosub/guppy/util/bin/prequal");
            // });
            // t.detach();
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    const auto nav_start_node = std::make_shared<NavStart>();
    rclcpp::spin(nav_start_node);
    rclcpp::shutdown();
    return 0;
}
