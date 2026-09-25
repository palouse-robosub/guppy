#ifndef POSE_SETTER_BEHAVIOR_HPP
#define POSE_SETTER_BEHAVIOR_HPP

#include "guppy_msgs/action/navigate.hpp"

#include <behaviortree_ros2/bt_action_node.hpp>

class NavigateBehavior
: public BT::RosActionNode<guppy_msgs::action::Navigate> {
public:
    NavigateBehavior(
        const std::string& name, const BT::NodeConfig& config,
        const BT::RosNodeParams& parameters
    );
    static BT::PortsList providedPorts();
    bool setGoal(
        BT::RosActionNode<guppy_msgs::action::Navigate>::Goal& goal
    ) override;
    BT::NodeStatus onResultReceived(const WrappedResult& wrapped) override;
    virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;
};

#endif
