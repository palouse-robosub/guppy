#pragma once

#include "guppy_msgs/action/navigate.hpp"

#include <behaviortree_ros2/bt_action_node.hpp>

class NavigateBehavior: public BT::RosActionNode<guppy_msgs::action::Navigate> {
public:
    NavigateBehavior(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : BT::RosActionNode<guppy_msgs::action::Navigate>(name, conf, params) {
        RCLCPP_INFO(logger(), "PoseSetter behavior initialized.");
    }

    static BT::PortsList providedPorts() {
        return providedBasicPorts({
            BT::InputPort<double>("x"), BT::InputPort<double>("y"), BT::InputPort<double>("z"),
            BT::InputPort<double>("qw"), BT::InputPort<double>("qx"), BT::InputPort<double>("qy"), BT::InputPort<double>("qz"),
            BT::InputPort<bool>("local"), BT::InputPort<double>("timeout"),
            BT::InputPort<bool>("continueOnTimeout")
        });
    }

    bool setGoal(BT::RosActionNode<guppy_msgs::action::Navigate>::Goal& goal) override {
        getInput("x", goal.pose.position.x);
        getInput("y", goal.pose.position.y);
        getInput("z", goal.pose.position.z);
        getInput("qw", goal.pose.orientation.w);
        getInput("qx", goal.pose.orientation.x);
        getInput("qy", goal.pose.orientation.y);
        getInput("qz", goal.pose.orientation.z);
        getInput("local", goal.local);
        getInput("timeout", goal.timeout);
        return true;
    }

    BT::NodeStatus onResultReceived(const WrappedResult& wrapped) override {
        RCLCPP_INFO(logger(), "PoseSetter action server returned results. %s reach target, with perror (%lf, %lf, %lf) qerror (%lf, %lf, %lf, %lf)", wrapped.result->target_reached ? "DID" : "DID NOT", wrapped.result->error.position.x, wrapped.result->error.position.y, wrapped.result->error.position.z, wrapped.result->error.orientation.w, wrapped.result->error.orientation.x, wrapped.result->error.orientation.y, wrapped.result->error.orientation.z);
        return BT::NodeStatus::SUCCESS;
    }

    virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override {
        bool continueOnTimeout;
        getInput("continueOnTimeout", continueOnTimeout);
        if (continueOnTimeout) {
            RCLCPP_INFO(logger(), "PoseSetter action server returned %s, continuing.", BT::toStr(error));
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(logger(), "PoseSetter action server return %s. Node status failing.", BT::toStr(error));
            return BT::NodeStatus::FAILURE;
        }
    }

    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) {
        return BT::NodeStatus::RUNNING;
    }
};
