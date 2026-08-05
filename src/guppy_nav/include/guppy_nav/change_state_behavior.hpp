#pragma once

#include <behaviortree_ros2/bt_service_node.hpp>
#include <guppy_msgs/srv/change_state.hpp>

class ChangeStateBehavior : public BT::RosServiceNode<guppy_msgs::srv::ChangeState>
{
public:
    ChangeStateBehavior(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : RosServiceNode<guppy_msgs::srv::ChangeState>(name, conf, params) {
        RCLCPP_INFO(logger(), "ChangeState behavior initialized.");
    }

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::InputPort<uint8_t>("state")
        });
    }

    bool setRequest(Request::SharedPtr& request) override
    {
        getInput("state", request->new_state.state);

        RCLCPP_INFO(logger(), "Requesting state change to '%u'.", static_cast<unsigned int>(request->new_state.state));

        return true;
    }

    BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override
    {
        RCLCPP_INFO(logger(), "%s response received from ChangeState action server.", response->success ? "OK" : "BAD");

        return BT::NodeStatus::SUCCESS;
    }
};
