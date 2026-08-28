#include <algorithm>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include "guppy_msgs/msg/corner_detection.hpp"
#include "guppy_msgs/msg/corner_detection_list.hpp"

class AcquireDetection
    : public BT::RosTopicSubNode<guppy_msgs::msg::CornerDetectionList> {
 public:
  AcquireDetection(const std::string& name,
                   const BT::NodeConfig& config,
                   const BT::RosNodeParams& params)
      : BT::RosTopicSubNode<guppy_msgs::msg::CornerDetectionList>(name,
                                                                  config,
                                                                  params) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<std::string>("target"),
         BT::OutputPort<guppy_msgs::msg::CornerDetection>("detection")});
  }

  BT::NodeStatus onTick(
      const std::shared_ptr<guppy_msgs::msg::CornerDetectionList>& msg)
      override {
    if (!msg)
      return BT::NodeStatus::FAILURE;  // no detections in list

    std::string target;
    getInput("target", target);

    auto it = std::find_if(
        msg->detections.begin(), msg->detections.end(),
        [target](const guppy_msgs::msg::CornerDetection detection) {
          return matchesTarget(detection, target);
        });
    if (it == msg->detections.end())
      return BT::NodeStatus::FAILURE;  // no detection matching target

    setOutput("detection", *it);

    return BT::NodeStatus::SUCCESS;
  }

 private:
  static bool matchesTarget(const guppy_msgs::msg::CornerDetection detection,
                            const std::string& target) {
    return detection.name == target;
  }
};
