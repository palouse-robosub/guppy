#include "guppy_nav/acquire_detection.hpp"

//  public methods
AcquireDetection::AcquireDetection(
    const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& parameters
) : BT::RosTopicSubNode<guppy_msgs::msg::CornerDetectionList>(name, config, parameters) { }

BT::PortsList AcquireDetection::providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<std::string>("target"),
         BT::OutputPort<guppy_msgs::msg::CornerDetection>("detection")}
    );
}

BT::NodeStatus
    AcquireDetection::onTick(const std::shared_ptr<guppy_msgs::msg::CornerDetectionList>& msg) {
    if (!msg)
        return BT::NodeStatus::FAILURE;    // no detections in list

    std::string target;
    this->getInput("target", target);

    auto it = std::find_if(
        msg->detections.begin(), msg->detections.end(),
        [&target](const guppy_msgs::msg::CornerDetection detection) {
            return AcquireDetection::matchesTarget(detection, target);
        }
    );
    if (it == msg->detections.end())
        return BT::NodeStatus::FAILURE;    // no detection matching target

    this->setOutput("detection", *it);

    return BT::NodeStatus::SUCCESS;
}

// private methods
bool AcquireDetection::matchesTarget(
    const guppy_msgs::msg::CornerDetection detection, std::string_view target
) {
    return detection.name == target;
}
