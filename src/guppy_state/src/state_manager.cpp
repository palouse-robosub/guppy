#include <chrono>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "guppy_msgs/srv/change_state.hpp"
#include "guppy_msgs/msg/state.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "guppy_msgs/msg/can_frame.hpp"


using namespace std::chrono_literals;

class StateManager : public rclcpp::Node {
    public:
        StateManager() : Node("state_manager"), current_state_(guppy_msgs::msg::State::STARTUP) {
            auto state_quality = rclcpp::QoS(1);
            state_quality.reliable();
            state_quality.transient_local();
            state_quality.keep_last(1);

            estopsubscription_ = this->create_subscription<guppy_msgs::msg::CanFrame>(
                "/can/id_0x1b", 10,
                std::bind(&StateManager::estopcallback, this, std::placeholders::_1)
            );
            resetholdpose = this->create_client<std_srvs::srv::Empty>("reset_holding_pose");

            state_publisher_ = this->create_publisher<guppy_msgs::msg::State>("state", state_quality); // ROS2 QoS let's you tell the topic to hold onto the last published state and ensure every node gets the state :)))))))
            state_service_ = this->create_service<guppy_msgs::srv::ChangeState>(
                "change_state",
                std::bind(&StateManager::transition_callback, this, std::placeholders::_1, std::placeholders::_2)
            );
            timer_ = this->create_wall_timer(1ms, std::bind(&StateManager::on_timer, this));

            auto nav_callback       = [this](geometry_msgs::msg::Twist::UniquePtr msg) -> void { this->nav_twist_ = *msg; };
            auto task_callback      = [this](geometry_msgs::msg::Twist::UniquePtr msg) -> void { this->task_twist_ = *msg; };
            auto teleop_callback    = [this](geometry_msgs::msg::Twist::UniquePtr msg) -> void { this->teleop_twist_ = *msg; };

            this->nav_subscription_     = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel/nav", 10, nav_callback);
            this->task_subscription_    = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel/task", 10, task_callback);
            this->teleop_subscription_  = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel/teleop", 10, teleop_callback);

            cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); // this one too

            static const geometry_msgs::msg::Twist zero_twist;
        }
    private:
        void estopcallback(guppy_msgs::msg::CanFrame msg) {
            int is_estopped = 0;
            memcpy(&is_estopped, msg.data.data(), sizeof(int));
            if (is_estopped) {
                this->publish_state(guppy_msgs::msg::State::DISABLED);
                was_estopped = true;
            }
            // else if (!is_estopped && was_estopped) {
            //     this->publish_state(guppy_msgs::msg::State::DISABLED);
            //     was_estopped = false;
            // }
        }

        static std::string to_string(uint8_t state) {
            switch (state) {
                case guppy_msgs::msg::State::STARTUP:  return "STARTUP";  break;
                case guppy_msgs::msg::State::HOLDING:  return "HOLDING";  break;
                case guppy_msgs::msg::State::NAV:      return "NAV";      break;
                case guppy_msgs::msg::State::TASK:     return "TASK";     break;
                case guppy_msgs::msg::State::TELEOP:   return "TELEOP";   break;
                case guppy_msgs::msg::State::DISABLED: return "DISABLED"; break;
                case guppy_msgs::msg::State::FAULT:    return  "FAULT";    break;
            }
        }

         bool is_valid_state(uint8_t state) {
            switch (state) {
                case guppy_msgs::msg::State::STARTUP:
                case guppy_msgs::msg::State::HOLDING:
                case guppy_msgs::msg::State::NAV:
                case guppy_msgs::msg::State::TASK:
                case guppy_msgs::msg::State::TELEOP:
                case guppy_msgs::msg::State::DISABLED:
                case guppy_msgs::msg::State::FAULT:
                    return true;
                default:
                    return false;
            }
        }

        void transition_callback(
            const std::shared_ptr<guppy_msgs::srv::ChangeState::Request> request,
            std::shared_ptr<guppy_msgs::srv::ChangeState::Response> response
        ) {
            RCLCPP_INFO(get_logger(), "State transition to %s requested.", to_string(request->new_state.state).c_str());

            auto new_state = request->new_state.state;

            if (new_state == this->current_state_) {
                RCLCPP_WARN(this->get_logger(), "Already in state %s!", to_string(current_state_).c_str());
                response->success = false;
                return;
            }

            if (!is_valid_state(new_state)) {
                RCLCPP_ERROR(this->get_logger(), "Invalid state passed in transition service!");
                response->success = false;
                return;
            }

            if (this->current_state_ == guppy_msgs::msg::State::FAULT) {
                RCLCPP_WARN(this->get_logger(), "You can't exit the FAULT state!");
                response->success = false;
                return;
            }

            if (new_state == guppy_msgs::msg::State::HOLDING) {
                RCLCPP_ERROR(this->get_logger(), "Resting pose for holding.");
                auto request = std::make_shared<std_srvs::srv::Empty::Request>();
                resetholdpose->async_send_request(request);
            }

            // TODO switch logic should be handled here NOT in StateManager#publishState()

            auto stale_state = current_state_;

            response->success = this->publish_state(new_state);

            if (response->success) RCLCPP_INFO(this->get_logger(), "Transitioning state from %s -> %s.", to_string(stale_state).c_str(), to_string(new_state).c_str());
            else RCLCPP_ERROR(get_logger(), "Failed to publish state transition from %s -> %s.", to_string(stale_state).c_str(), to_string(new_state).c_str());
        }

        bool publish_state(uint8_t state) {
            auto message = guppy_msgs::msg::State();
            message.state = state;
            this->cmd_vel_publisher_->publish(StateManager::zero_twist);
            this->state_publisher_->publish(message);
            this->current_state_ = state;
            return true;
        }

        void on_timer() {
            switch (this->current_state_) {
                case guppy_msgs::msg::State::STARTUP:  this->handle_startup();  break;
                case guppy_msgs::msg::State::HOLDING:  this->handle_holding();  break;
                case guppy_msgs::msg::State::NAV:      this->handle_nav();      break;
                case guppy_msgs::msg::State::TASK:     this->handle_task();     break;
                case guppy_msgs::msg::State::TELEOP:   this->handle_teleop();   break;
                case guppy_msgs::msg::State::DISABLED: this->handle_disabled(); break;
                case guppy_msgs::msg::State::FAULT:    this->handle_fault();    break;
            }
        }

        // state handlers
        void handle_startup() {
            // start disabled (for now for testing at least)
            this->publish_state(guppy_msgs::msg::State::DISABLED);
        }

        void handle_holding() {
            this->cmd_vel_publisher_->publish(StateManager::zero_twist); // does this need to constantly publish? if not, just publish once upon transition to holding.
        }

        void handle_nav() {
            if (this->nav_twist_.has_value())
                this->cmd_vel_publisher_->publish(nav_twist_.value());
        }

        void handle_task() {
            if (this->task_twist_.has_value())
                this->cmd_vel_publisher_->publish(task_twist_.value());
        }

        void handle_teleop() {
            if (this->teleop_twist_.has_value())
                this->cmd_vel_publisher_->publish(teleop_twist_.value());
        }

        void handle_disabled() {
            // system("killall prequal");
            this->publish_state(guppy_msgs::msg::State::DISABLED);
        }

        void handle_fault() {
            // TODO
        }

        uint8_t current_state_;
        rclcpp::Publisher<guppy_msgs::msg::State>::SharedPtr state_publisher_;
        rclcpp::Subscription<guppy_msgs::msg::CanFrame>::SharedPtr estopsubscription_;
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr resetholdpose;

        rclcpp::Service<guppy_msgs::srv::ChangeState>::SharedPtr state_service_;
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr task_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_subscription_;

        std::optional<geometry_msgs::msg::Twist> nav_twist_;
        std::optional<geometry_msgs::msg::Twist> task_twist_;
        std::optional<geometry_msgs::msg::Twist> teleop_twist_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

        bool was_estopped = false;

    const geometry_msgs::msg::Twist zero_twist = []() {
        geometry_msgs::msg::Vector3 zero_vector;
        zero_vector.x = 0.0;
        zero_vector.y = 0.0;
        zero_vector.z = 0.0;

        geometry_msgs::msg::Twist twist;
        twist.linear = zero_vector;
        twist.angular = zero_vector;
        return twist;
    }();
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto publisher_node = std::make_shared<StateManager>();

    rclcpp::spin(publisher_node);

    rclcpp::shutdown();

    return 0;
}
