#include <cmath>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <control_toolbox/control_toolbox/pid.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "guppy_msgs/action/navigate.hpp"
#include "guppy_msgs/msg/state.hpp"
#include "guppy_msgs/srv/set_hold_pose.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"

#define SETPOINTS_EVERY 0.3 // meters

class PoseSetterServer : public rclcpp::Node {
public:
    explicit PoseSetterServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("pose_setter", options) {
        auto handleGoal = [this](const rclcpp_action::GoalUUID& id, std::shared_ptr<const guppy_msgs::action::Navigate::Goal> goal) {
            RCLCPP_INFO(this->get_logger(), "Goal %s requested at (%.2lf, %.1lf, %lf)(%.2lf, %.2lf, %.2lf, %.2lf) %s with %lf timeout.", rclcpp_action::to_string(id).c_str(), goal->pose.position.x, goal->pose.position.y, goal->pose.position.z, goal->pose.orientation.w, goal->pose.orientation.x, goal->pose.orientation.y, goal->pose.orientation.z, goal->local ? "LOCAL" : "ABSOLUTE", goal->timeout);

            if (_state == guppy_msgs::msg::State::NAV) {
                _cancel = false;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }

            RCLCPP_INFO(this->get_logger(), "Rejecting request because state not NAV."); //goal->pose
            return rclcpp_action::GoalResponse::REJECT;
        };

        auto handleCancel = [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<guppy_msgs::action::Navigate>> goalHandle) {
            RCLCPP_INFO(this->get_logger(), "Request to cancel goal %s.", rclcpp_action::to_string(goalHandle->get_goal_id()).c_str());
            this->_cancel = true;
            return rclcpp_action::CancelResponse::ACCEPT;
        };

        auto handleAccepted = [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<guppy_msgs::action::Navigate>> goalHandle) {
            RCLCPP_INFO(this->get_logger(), "Accepted goal %s.", rclcpp_action::to_string(goalHandle->get_goal_id()).c_str());
            std::thread{[this, goalHandle]() {
                execute(goalHandle);
            }}.detach();
        };

        _actionServer = rclcpp_action::create_server<guppy_msgs::action::Navigate>(
            this,
            "/navigate",
            handleGoal,
            handleCancel,
            handleAccepted,
            rcl_action_server_get_default_options()
        );
        _setter = this->create_client<guppy_msgs::srv::SetHoldPose>("reset_holding_pose");
        _odomSubscription = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered",10,std::bind(&PoseSetterServer::odometryCallback, this, std::placeholders::_1));
        _stateSubscription = this->create_subscription<guppy_msgs::msg::State>("/state",10,std::bind(&PoseSetterServer::stateCallback, this, std::placeholders::_1));
    }

    void odometryCallback(nav_msgs::msg::Odometry::SharedPtr msg) {
        _current_pos = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        _current_quat = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    }

    void stateCallback(guppy_msgs::msg::State::SharedPtr msg) {
        _state = msg->state;
    }
private:
    rclcpp_action::Server<guppy_msgs::action::Navigate>::SharedPtr _actionServer;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odomSubscription;
    rclcpp::Subscription<guppy_msgs::msg::State>::SharedPtr _stateSubscription;
    rclcpp::Client<guppy_msgs::srv::SetHoldPose>::SharedPtr _setter;

    Eigen::Vector3d _current_pos;
    Eigen::Quaterniond _current_quat;
    int _state = 0;
    bool _cancel = false;

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
        return pose_from_vec_quat(_current_pos, _current_quat);
    }

    void set_to(Eigen::Vector3d vec, Eigen::Quaterniond quat) {
        auto request = std::make_shared<guppy_msgs::srv::SetHoldPose::Request>();
        request->pose = pose_from_vec_quat(vec, quat);
        request->type = request->GLOBAL;
        _setter->async_send_request(request);
    }

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<guppy_msgs::action::Navigate>> goalHandle) {
        const auto& goal = goalHandle->get_goal();
        Eigen::Vector3d _end_pos(goal->pose.position.x, goal->pose.position.y, goal->pose.position.z);
        Eigen::Quaterniond _end_quat(goal->pose.orientation.w, goal->pose.orientation.x, goal->pose.orientation.y, goal->pose.orientation.z);

        if (goal->local) {
            _end_pos = _current_pos + (_current_quat.inverse() * _end_pos);
            _end_quat = _current_quat * _end_quat;
        }

        Eigen::Vector3d _start_pos = _current_pos;
        Eigen::Quaterniond _start_quat = _current_quat;


        Eigen::Vector3d dist_vector = _end_pos - _start_pos;

        double total_distance = dist_vector.norm();

        int n_steps = (total_distance / SETPOINTS_EVERY) + 1;
        std::vector<Eigen::Vector3d> pos_list;
        std::vector<Eigen::Quaterniond> quat_list;
        std::vector<double> tolerance_list;

        for (int i=1; i<=n_steps; i++) {
            double t = ((double)i) / n_steps;
            pos_list.push_back((1.0 - t) * _start_pos + t * _end_pos);
            quat_list.push_back(_start_quat.slerp(t, _end_quat));
            tolerance_list.push_back(SETPOINTS_EVERY);
        }

        tolerance_list[n_steps - 1] = goal->tolerance;


        auto clock = this->get_clock();
        rclcpp::Time start = clock->now();

        set_to(pos_list[0], quat_list[0]);

        auto feedback = std::make_shared<guppy_msgs::action::Navigate::Feedback>();
        auto result = std::make_shared<guppy_msgs::action::Navigate::Result>();

        rclcpp::Rate rate(50);

        int setpoint_index = 0;

        Eigen::Vector3d error;
        Eigen::Quaterniond qerror;

        while (rclcpp::ok()) {
            if (_cancel || goalHandle->is_canceling() || _state != guppy_msgs::msg::State::NAV) {
                result->pose = get_current_pose();
                result->target_reached = false;
                goalHandle->canceled(result);
                return;
            }

            error = pos_list[setpoint_index] - _current_pos;
            qerror = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

            auto qdistance = _current_quat.inverse() * quat_list[setpoint_index];
            auto qangle = 2 * atan2(qdistance.vec().norm(), qdistance.w());

            if (
                abs(error.x()) <= tolerance_list[setpoint_index] &&
                abs(error.y()) <= tolerance_list[setpoint_index] &&
                abs(error.z()) <= tolerance_list[setpoint_index] &&
                abs(qangle) <= M_PI / 36 // 10 degrees
            ) {
                setpoint_index++;
                if (setpoint_index == n_steps) {
                    break;
                }
                set_to(pos_list[setpoint_index], quat_list[setpoint_index]);
            }

            else if ((clock->now() - start).seconds() >= goal->timeout) {
                result->pose = get_current_pose();
                result->error = pose_from_vec_quat(error, qerror);
                result->target_reached = false;
                goalHandle->abort(result);
                return;
            }

            feedback->progress = get_current_pose();
            feedback->percent_done = ((double)setpoint_index) / ((double)n_steps);
            goalHandle->publish_feedback(feedback);
            rate.sleep();
        }

        if (rclcpp::ok()) {
            result->pose = get_current_pose();
            result->target_reached = true;
            result->error = pose_from_vec_quat(error, qerror);
            goalHandle->succeed(result);
            return;
        }
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(PoseSetterServer)

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PoseSetterServer>();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);

    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
