#ifndef MOVE_TOWARD_BEHAVIOR_HPP
#define MOVE_TOWARD_BEHAVIOR_HPP

#include <behaviortree_ros2/bt_action_node.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "guppy_msgs/action/navigate.hpp"

class MoveTowardBehavior : public BT::RosActionNode<guppy_msgs::action::Navigate> {
public:
    MoveTowardBehavior(
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
