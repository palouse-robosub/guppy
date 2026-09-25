#include "guppy_nav/pose_setter_behavior.hpp"

#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"

// public methods
NavigateBehavior::NavigateBehavior(
    const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& parameters
) : BT::RosActionNode<guppy_msgs::action::Navigate>(name, config, parameters) {
    RCLCPP_INFO(this->logger(), "PoseSetter behavior initialized.");
}

BT::PortsList NavigateBehavior::providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<double>("x"), BT::InputPort<double>("y"), BT::InputPort<double>("z"),
         BT::InputPort<double>("qw"), BT::InputPort<double>("qx"), BT::InputPort<double>("qy"),
         BT::InputPort<double>("qz"), BT::InputPort<bool>("local"),
         BT::InputPort<double>("timeout"), BT::InputPort<bool>("continueOnTimeout")}
    );
}

bool NavigateBehavior::setGoal(BT::RosActionNode<guppy_msgs::action::Navigate>::Goal& goal) {
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

BT::NodeStatus NavigateBehavior::onResultReceived(const WrappedResult& wrapped) {
    RCLCPP_INFO(
        this->logger(),
        "PoseSetter action server returned results. %s reach target, with "
        "perror (%lf, %lf, %lf) qerror (%lf, %lf, %lf, %lf)",
        wrapped.result->target_reached ? "DID" : "DID NOT", wrapped.result->error.position.x,
        wrapped.result->error.position.y, wrapped.result->error.position.z,
        wrapped.result->error.orientation.w, wrapped.result->error.orientation.x,
        wrapped.result->error.orientation.y, wrapped.result->error.orientation.z
    );
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus NavigateBehavior::onFailure(BT::ActionNodeErrorCode error) {
    bool continueOnTimeout;
    this->getInput("continueOnTimeout", continueOnTimeout);
    if (continueOnTimeout) {
        RCLCPP_INFO(
            this->logger(), "PoseSetter action server returned %s, continuing.", BT::toStr(error)
        );
        return BT::NodeStatus::SUCCESS;
    } else {
        RCLCPP_ERROR(
            this->logger(), "PoseSetter action server return %s. Node status failing.",
            BT::toStr(error)
        );
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus NavigateBehavior::onFeedback(const std::shared_ptr<const Feedback>) {
    return BT::NodeStatus::RUNNING;
}
