#include <cmath>
#include <memory>
#include <thread>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <control_toolbox/control_toolbox/pid.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "guppy_msgs/action/navigate.hpp"
#include "guppy_msgs/msg/state.hpp"
#include "guppy_msgs/srv/set_hold_pose.hpp"
#include "guppy_util/quality.hpp"

class PoseSetterServer : public rclcpp::Node {
private:
    static constexpr const inline auto point_spacing = 0.3; // meters
private:
    std::shared_ptr<rclcpp_action::Server<guppy_msgs::action::Navigate>>
                                                             action_server_;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> odom_sub_;
    std::shared_ptr<rclcpp::Subscription<guppy_msgs::msg::State>>  state_sub_;
    std::shared_ptr<rclcpp::Client<guppy_msgs::srv::SetHoldPose>>  setter_;

    Eigen::Vector3d    current_pos_;
    Eigen::Quaterniond current_quat_;
    int                state_  = 0;
    bool               cancel_ = false;
public:
    explicit PoseSetterServer(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    ) : Node("pose_setter", options) {
        auto handle_goal =
            [this](
                const rclcpp_action::GoalUUID&                            id,
                std::shared_ptr<const guppy_msgs::action::Navigate::Goal> goal
            ) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Goal %s requested at (%.2lf, %.1lf, %lf)(%.2lf, %.2lf, "
                    "%.2lf, %.2lf) %s with %lf timeout.",
                    rclcpp_action::to_string(id).c_str(), goal->pose.position.x,
                    goal->pose.position.y, goal->pose.position.z,
                    goal->pose.orientation.w, goal->pose.orientation.x,
                    goal->pose.orientation.y, goal->pose.orientation.z,
                    goal->local ? "LOCAL" : "ABSOLUTE", goal->timeout
                );

                if (state_ == guppy_msgs::msg::State::NAV) {
                    cancel_ = false;
                    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
                }

                RCLCPP_INFO(
                    this->get_logger(),
                    "Rejecting request because state not NAV."
                );    // goal->pose
                return rclcpp_action::GoalResponse::REJECT;
            };

        auto handle_cancel =
            [this](
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                    guppy_msgs::action::Navigate>>
                    goal_handle
            ) {
                RCLCPP_INFO(
                    this->get_logger(), "Request to cancel goal %s.",
                    rclcpp_action::to_string(goal_handle->get_goal_id()).c_str()
                );
                this->cancel_ = true;
                return rclcpp_action::CancelResponse::ACCEPT;
            };

        auto handle_accepted =
            [this](
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                    guppy_msgs::action::Navigate>>
                    goal_handle
            ) {
                RCLCPP_INFO(
                    this->get_logger(), "Accepted goal %s.",
                    rclcpp_action::to_string(goal_handle->get_goal_id()).c_str()
                );
                std::thread{
                    [this, goal_handle]() {
                        execute(goal_handle);
                    }
                }.detach();
            };

        action_server_ =
            rclcpp_action::create_server<guppy_msgs::action::Navigate>(
                this, "/navigate", handle_goal, handle_cancel, handle_accepted,
                rcl_action_server_get_default_options()
            );
        setter_ = this->create_client<guppy_msgs::srv::SetHoldPose>(
            "reset_holding_pose"
        );
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", quality::volatile_profile,
            [this](std::shared_ptr<nav_msgs::msg::Odometry> msg) {
                this->odometry_callback(*msg);
            }
        );
        state_sub_ = this->create_subscription<guppy_msgs::msg::State>(
            "/state", quality::keep_last_profile,
            [this](const std::shared_ptr<const guppy_msgs::msg::State> msg) {
                state_callback(*msg);
            }
        );
    }

private:
    void odometry_callback(const nav_msgs::msg::Odometry& msg) {
        current_pos_ = Eigen::Vector3d(
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            msg.pose.pose.position.z
        );
        current_quat_ = Eigen::Quaterniond(
            msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y, msg.pose.pose.orientation.z
        );
    }

    void state_callback(const guppy_msgs::msg::State& msg) {
        state_ = msg.state;
    }

    geometry_msgs::msg::Pose pose_from_vec_quat(Eigen::Vector3d vec, Eigen::Quaterniond quat) {
        geometry_msgs::msg::Pose out;
        out.orientation.w = quat.w();
        out.orientation.x = quat.x();
        out.orientation.y = quat.y();
        out.orientation.z = quat.z();
        out.position.x    = vec.x();
        out.position.y    = vec.y();
        out.position.z    = vec.z();
        return out;
    }

    geometry_msgs::msg::Pose get_current_pose() {
        return pose_from_vec_quat(current_pos_, current_quat_);
    }

    void set_to(Eigen::Vector3d vec, Eigen::Quaterniond quat) {
        auto request =
            std::make_shared<guppy_msgs::srv::SetHoldPose::Request>();
        request->pose = pose_from_vec_quat(vec, quat);
        request->type = request->GLOBAL;
        setter_->async_send_request(request);
    }

    void execute(
        const std::shared_ptr<
            rclcpp_action::ServerGoalHandle<guppy_msgs::action::Navigate>>
            goal_handle
    ) {
        const auto&     goal = goal_handle->get_goal();
        Eigen::Vector3d end_pos(
            goal->pose.position.x, goal->pose.position.y, goal->pose.position.z
        );
        Eigen::Quaterniond end_quat(
            goal->pose.orientation.w, goal->pose.orientation.x,
            goal->pose.orientation.y, goal->pose.orientation.z
        );

        if (goal->local) {
            end_pos  = current_pos_ + (current_quat_.inverse() * end_pos);
            end_quat = current_quat_ * end_quat;
        }

        Eigen::Vector3d    start_pos  = current_pos_;
        Eigen::Quaterniond start_quat = current_quat_;

        Eigen::Vector3d dist_vector = end_pos - start_pos;

        double total_distance = dist_vector.norm();

        int n_steps = (total_distance / PoseSetterServer::point_spacing) + 1;
        std::vector<Eigen::Vector3d>    pos_list;
        std::vector<Eigen::Quaterniond> quat_list;
        std::vector<double>             tolerance_list;

        for (int i = 1; i <= n_steps; i++) {
            double t = ((double)i) / n_steps;
            pos_list.push_back((1.0 - t) * start_pos + t * end_pos);
            quat_list.push_back(start_quat.slerp(t, end_quat));
            tolerance_list.push_back(PoseSetterServer::point_spacing);
        }

        tolerance_list[n_steps - 1] = goal->tolerance;

        auto         clock = this->get_clock();
        rclcpp::Time start = clock->now();

        set_to(pos_list[0], quat_list[0]);

        auto feedback =
            std::make_shared<guppy_msgs::action::Navigate::Feedback>();
        auto result = std::make_shared<guppy_msgs::action::Navigate::Result>();

        rclcpp::Rate rate(50);

        int setpoint_index = 0;

        Eigen::Vector3d    error = {0, 0, 0};
        Eigen::Quaterniond qerror = {0, 0, 0, 1};

        while (rclcpp::ok()) {
            if (cancel_ || goal_handle->is_canceling()
                || state_ != guppy_msgs::msg::State::NAV) {
                result->pose           = get_current_pose();
                result->target_reached = false;
                goal_handle->canceled(result);
                return;
            }

            error  = pos_list[setpoint_index] - this->current_pos_;
            qerror = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

            auto qdistance =
                current_quat_.inverse() * quat_list[setpoint_index];
            auto qangle = 2 * atan2(qdistance.vec().norm(), qdistance.w());

            if (
                abs(error.x()) <= tolerance_list[setpoint_index]
                && abs(error.y()) <= tolerance_list[setpoint_index]
                && abs(error.z()) <= tolerance_list[setpoint_index]
                && abs(qangle) <= M_PI / 36    // 10 degrees
            ) {
                setpoint_index++;
                if (setpoint_index == n_steps)
                    break;
                set_to(pos_list[setpoint_index], quat_list[setpoint_index]);
            } else if ((clock->now() - start).seconds() >= goal->timeout) {
                result->pose           = get_current_pose();
                result->error          = pose_from_vec_quat(error, qerror);
                result->target_reached = false;
                goal_handle->abort(result);
                return;
            }

            feedback->progress = get_current_pose();
            feedback->percent_done =
                ((double)setpoint_index) / ((double)n_steps);
            goal_handle->publish_feedback(feedback);
            rate.sleep();
        }

        if (rclcpp::ok()) {
            result->pose           = get_current_pose();
            result->target_reached = true;
            result->error          = pose_from_vec_quat(error, qerror);
            goal_handle->succeed(result);
            return;
        }
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(PoseSetterServer)

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<PoseSetterServer>();
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 2
    );
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
