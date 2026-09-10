#include <memory>
#include <optional>

#include "guppy_msgs/msg/can_frame.hpp"
#include "guppy_msgs/msg/state.hpp"
#include "guppy_msgs/srv/change_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/empty.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/executors.hpp"

using namespace std::chrono_literals;

class StateManager : public rclcpp::Node {
public:
    static inline const auto keep_last_profile = rclcpp::QoS(10).reliable().transient_local().keep_last(1);
    static inline const auto reliable_profile = rclcpp::QoS(10).reliable();
    static inline const auto volatile_profile = rclcpp::QoS(10).best_effort().durability_volatile();
    static inline const geometry_msgs::msg::Twist zero_twist{};
private:
    uint8_t                                              current_state_;
    rclcpp::Publisher<guppy_msgs::msg::State>::SharedPtr state_pub_;
    rclcpp::Subscription<guppy_msgs::msg::CanFrame>::SharedPtr
                                                    emergency_stop_sub_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_pose_;

    rclcpp::Service<guppy_msgs::srv::ChangeState>::SharedPtr state_service_;
    rclcpp::TimerBase::SharedPtr                             timer_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr task_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;

    std::optional<geometry_msgs::msg::Twist> nav_twist_;
    std::optional<geometry_msgs::msg::Twist> task_twist_;
    std::optional<geometry_msgs::msg::Twist> teleop_twist_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    bool was_emergency_stopped_ = false;
public:
    StateManager() :
        Node("state_manager"), current_state_(guppy_msgs::msg::State::STARTUP) {

        this->emergency_stop_sub_ =
            this->create_subscription<guppy_msgs::msg::CanFrame>(
                "/can/id_0x1b", keep_last_profile,
                [this](const guppy_msgs::msg::CanFrame::SharedPtr& msg){
                    this->emergency_stop_callback(*msg);
                }
            );
        this->reset_pose_ = this->create_client<std_srvs::srv::Empty>("reset_holding_pose", reliable_profile);

        this->state_pub_ = this->create_publisher<guppy_msgs::msg::State>(
            "state", keep_last_profile
        );

        this->state_service_ = this->create_service<guppy_msgs::srv::ChangeState>(
            "change_state",
            [this](const std::shared_ptr<guppy_msgs::srv::ChangeState::Request>& request, const std::shared_ptr<guppy_msgs::srv::ChangeState::Response>& response){
                this->transition_callback(*request, *response);
            }
        );

        this->timer_ = this->create_wall_timer(
            1ms,
            [this](){
                this->on_timer();
            }
        );

        auto nav_callback =
            [this](const geometry_msgs::msg::Twist::UniquePtr& msg) {
            this->nav_twist_ = *msg;
        };
        auto task_callback =
            [this](const geometry_msgs::msg::Twist::UniquePtr& msg) {
            this->task_twist_ = *msg;
        };
        auto teleop_callback =
            [this](const geometry_msgs::msg::Twist::UniquePtr& msg) {
            this->teleop_twist_ = *msg;
        };

        this->nav_sub_ =
            this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel/nav", volatile_profile, nav_callback
            );
        this->task_sub_ =
            this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel/task", volatile_profile, task_callback
            );
        this->teleop_sub_ =
            this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel/teleop", volatile_profile, teleop_callback
            );

        this->cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", volatile_profile
        );
    }
  private:
    void emergency_stop_callback(const guppy_msgs::msg::CanFrame& msg) {
        int is_emergency_stopped = 0;
        memcpy(&is_emergency_stopped, msg.data.data(), sizeof(int));
        if (is_emergency_stopped) {
            this->publish_state(guppy_msgs::msg::State::DISABLED);
            this->was_emergency_stopped_ = true;
        }
    }

    static std::string to_string(uint8_t state) {
        switch (state) {
        case guppy_msgs::msg::State::STARTUP:  return "STARTUP"; break;
        case guppy_msgs::msg::State::HOLDING:  return "HOLDING"; break;
        case guppy_msgs::msg::State::NAV:      return "NAV"; break;
        case guppy_msgs::msg::State::TASK:     return "TASK"; break;
        case guppy_msgs::msg::State::TELEOP:   return "TELEOP"; break;
        case guppy_msgs::msg::State::DISABLED: return "DISABLED"; break;
        case guppy_msgs::msg::State::FAULT:    return "FAULT"; break;
        }
    }

    static bool is_valid_state(uint8_t state) {
        switch (state) {
        case guppy_msgs::msg::State::STARTUP:
        case guppy_msgs::msg::State::HOLDING:
        case guppy_msgs::msg::State::NAV:
        case guppy_msgs::msg::State::TASK:
        case guppy_msgs::msg::State::TELEOP:
        case guppy_msgs::msg::State::DISABLED:
        case guppy_msgs::msg::State::FAULT:    return true;
        default:                               return false;
        }
    }

    void transition_callback(
        const guppy_msgs::srv::ChangeState::Request& request,
        guppy_msgs::srv::ChangeState::Response&      response
    ) {
        RCLCPP_INFO(
            get_logger(), "State transition to %s requested.",
            to_string(request.new_state.state).c_str()
        );

        auto new_state = request.new_state.state;

        if (new_state == this->current_state_) {
            RCLCPP_WARN(
                this->get_logger(), "Already in state %s!",
                to_string(current_state_).c_str()
            );
            response.success = false;
            return;
        }

        if (!is_valid_state(new_state)) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Invalid state passed in transition service!"
            );
            response.success = false;
            return;
        }

        if (this->current_state_ == guppy_msgs::msg::State::FAULT) {
            RCLCPP_WARN(this->get_logger(), "You can't exit the FAULT state!");
            response.success = false;
            return;
        }

        if (new_state == guppy_msgs::msg::State::HOLDING) {
            RCLCPP_ERROR(this->get_logger(), "Resting pose for holding.");
            auto request = std::make_shared<std_srvs::srv::Empty::Request>();
            this->reset_pose_->async_send_request(request);
        }

        auto stale_state = current_state_;

        response.success = this->publish_state(new_state);

        if (response.success)
            RCLCPP_INFO(
                this->get_logger(), "Transitioning state from %s -> %s.",
                to_string(stale_state).c_str(), to_string(new_state).c_str()
            );
        else
            RCLCPP_ERROR(
                get_logger(),
                "Failed to publish state transition from %s -> %s.",
                to_string(stale_state).c_str(), to_string(new_state).c_str()
            );
    }

    bool publish_state(uint8_t state) {
        auto message  = guppy_msgs::msg::State();
        message.state = state;
        this->cmd_vel_pub_->publish(StateManager::zero_twist);
        this->state_pub_->publish(message);
        this->current_state_ = state;
        return true;
    }

    void on_timer() {
        switch (this->current_state_) {
        case guppy_msgs::msg::State::STARTUP:  this->handle_startup(); break;
        case guppy_msgs::msg::State::HOLDING:  this->handle_holding(); break;
        case guppy_msgs::msg::State::NAV:      this->handle_nav(); break;
        case guppy_msgs::msg::State::TASK:     this->handle_task(); break;
        case guppy_msgs::msg::State::TELEOP:   this->handle_teleop(); break;
        case guppy_msgs::msg::State::DISABLED: this->handle_disabled(); break;
        case guppy_msgs::msg::State::FAULT:    this->handle_fault(); break;
        }
    }

    // state handlers
    void handle_startup() {
        // start disabled (for now for testing at least)
        this->publish_state(guppy_msgs::msg::State::DISABLED);
    }

    void handle_holding() {
        this->cmd_vel_pub_->publish(
            StateManager::zero_twist
        );
    }

    void handle_nav() {
        if (this->nav_twist_.has_value())
            this->cmd_vel_pub_->publish(nav_twist_.value());
    }

    void handle_task() {
        if (this->task_twist_.has_value())
            this->cmd_vel_pub_->publish(task_twist_.value());
    }

    void handle_teleop() {
        if (this->teleop_twist_.has_value())
            this->cmd_vel_pub_->publish(teleop_twist_.value());
    }

    void handle_disabled() {
        this->publish_state(guppy_msgs::msg::State::DISABLED);
    }

    void handle_fault() {
        // TODO
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto publisher_node = std::make_shared<StateManager>();

    rclcpp::spin(publisher_node);

    rclcpp::shutdown();

    return 0;
}
