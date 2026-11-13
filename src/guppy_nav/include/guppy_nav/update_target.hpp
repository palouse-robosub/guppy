#pragma once

#include <optional>
#include <string>

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "guppy_msgs/msg/corner_detection.hpp"
#include "guppy_msgs/msg/corner_detection_list.hpp"

// SUCCESS: a detection matching `target` has been seen within `timeout`
//          seconds; the matching detection is emitted on `detection`.
// FAILURE: stale or never seen.
//
// Scope: freshness gate only.
class UpdateTarget :
  public BT::RosTopicSubNode<guppy_msgs::msg::CornerDetectionList> {
  public:
    UpdateTarget(
        const std::string& name, const BT::NodeConfig& config,
        const BT::RosNodeParams& params
    ) :
      BT::RosTopicSubNode<guppy_msgs::msg::CornerDetectionList>(
          name, config, params
      ) { }

    static BT::PortsList providedPorts() {
        return providedBasicPorts(
            {
              BT::InputPort<std::string>(
                  "target", "", "detection name to track ('' = first in list)"
              ),
              BT::InputPort<double>(
                  "timeout", 5.0,
                  "seconds without a matching detection before failing"
              ),
              BT::OutputPort<guppy_msgs::msg::CornerDetection>("detection"),
            }
        );
    }

    BT::NodeStatus onTick(
        const std::shared_ptr<guppy_msgs::msg::CornerDetectionList>& msg
    ) override {
        std::string target;
        double      timeout_s = 5.0;
        getInput("target", target);
        getInput("timeout", timeout_s);

        // Lock the weak_ptr we inherit from RosTopicSubNode.
        auto node = node_.lock();
        if (!node) {
            RCLCPP_ERROR(
                logger(),
                "UpdateTarget: ROS node went out of scope; cannot get clock."
            );
            return BT::NodeStatus::FAILURE;
        }
        const auto now = node->get_clock()->now();

        // 1) If a new detection list arrived, look for our target in it.
        if (msg) {
            for (const auto& det : msg->detections) {
                if (target.empty() || det.name == target) {
                    last_detection_ = det;
                    last_seen_      = now;
                    break;
                }
            }
        }

        // 2) Pass if we've seen the target within timeout range.
        if (last_seen_ && (now - *last_seen_).seconds() < timeout_s) {
            setOutput("detection", *last_detection_);
            RCLCPP_INFO(
                logger(), "UpdateTarget: '%s' fresh (age %.2fs < %.2fs)",
                last_detection_->name.c_str(), (now - *last_seen_).seconds(),
                timeout_s
            );
            return BT::NodeStatus::SUCCESS;
        }

        // 3) Stale or never seen: fail, let the tree fall back to search.
        RCLCPP_WARN(
            logger(), "UpdateTarget: no '%s' within %.2fs, failing to search",
            target.c_str(), timeout_s
        );
        return BT::NodeStatus::FAILURE;
    }

  private:
    std::optional<rclcpp::Time>                     last_seen_;
    std::optional<guppy_msgs::msg::CornerDetection> last_detection_;
};
