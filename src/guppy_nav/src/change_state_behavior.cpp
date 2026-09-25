#include "guppy_nav/change_state_behavior.hpp"

// public methods
ChangeStateBehavior::ChangeStateBehavior(
    const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& parameters
) : RosServiceNode<guppy_msgs::srv::ChangeState>(name, config, parameters) {
    RCLCPP_INFO(this->logger(), "ChangeState behavior initialized.");
}

BT::PortsList ChangeStateBehavior::providedPorts() {
    return providedBasicPorts({BT::InputPort<uint8_t>("state")});
}

bool ChangeStateBehavior::setRequest(std::shared_ptr<Request>& request) {
    this->getInput("state", request->new_state.state);

    RCLCPP_INFO(
        this->logger(), "Requesting state change to '%u'.",
        static_cast<unsigned int>(request->new_state.state)
    );

    return true;
}

BT::NodeStatus ChangeStateBehavior::onResponseReceived(const std::shared_ptr<Response>& response) {
    RCLCPP_INFO(
        this->logger(), "%s response received from ChangeState action server.",
        response->success ? "OK" : "BAD"
    );

    return BT::NodeStatus::SUCCESS;
}
