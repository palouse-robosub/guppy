#include <chrono>
#include <memory>
#include <optional>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "guppy_msgs/srv/change_state.hpp"
#include "guppy_msgs/msg/state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "guppy_msgs/msg/can_frame.hpp"


using namespace std::chrono_literals;

class NavStart : public rclcpp::Node {
    public:
        NavStart() : Node("navstart"){
            navswitchsub = this->create_subscription<guppy_msgs::msg::CanFrame>(
                "/can/id_0x22", 10,
                std::bind(&NavStart::switchcallback, this, std::placeholders::_1)
            );

            stateclient = this->create_client<guppy_msgs::srv::ChangeState>("change_state");
        }
    
    private:
        void switchcallback(guppy_msgs::msg::CanFrame msg) {
            int navon = 0;
            memcpy(&navon, msg.data.data(), sizeof(int));

            if (!initialized) {
                initialized = true;
                was_nav = navon;
                return;
            }

            if (!navon && was_nav) {
                auto request = std::make_shared<guppy_msgs::srv::ChangeState::Request>();
                guppy_msgs::msg::State state;
                state.state = guppy_msgs::msg::State::HOLDING;
                request->new_state = state;
                stateclient->async_send_request(request);
                was_nav = false;
            } else if (navon && !was_nav) {
                auto request = std::make_shared<guppy_msgs::srv::ChangeState::Request>();
                guppy_msgs::msg::State state;
                state.state = guppy_msgs::msg::State::NAV;
                request->new_state = state;
                stateclient->async_send_request(request);
                was_nav = true;

                // std::thread t([](){
                //     system("/home/robosub/guppy/util/bin/prequal");
                // });
                // t.detach();
                
            }
        }

        rclcpp::Subscription<guppy_msgs::msg::CanFrame>::SharedPtr navswitchsub;
        rclcpp::Client<guppy_msgs::srv::ChangeState>::SharedPtr stateclient;
        bool was_nav = false;
        bool initialized = false;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto publisher_node = std::make_shared<NavStart>();
    
    rclcpp::spin(publisher_node);


    rclcpp::shutdown();

    return 0;
}
