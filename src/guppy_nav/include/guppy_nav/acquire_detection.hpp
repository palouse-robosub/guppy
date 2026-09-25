#ifndef ACQUIRE_DETECTION_HPP
#define ACQUIRE_DETECTION_HPP

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

#include "guppy_msgs/msg/corner_detection.hpp"
#include "guppy_msgs/msg/corner_detection_list.hpp"
#include <behaviortree_ros2/bt_topic_sub_node.hpp>

class AcquireDetection : public BT::RosTopicSubNode<guppy_msgs::msg::CornerDetectionList> {
public:
    AcquireDetection(
        const std::string& name,
        const BT::NodeConfig& config,
        const BT::RosNodeParams& parameters
    );
    static BT::PortsList providedPorts();
    BT::NodeStatus onTick(
        const std::shared_ptr<guppy_msgs::msg::CornerDetectionList>& msg
    ) override;
private:
    static bool matchesTarget(
        const guppy_msgs::msg::CornerDetection detection,
        std::string_view                       target
    );
};

#endif
