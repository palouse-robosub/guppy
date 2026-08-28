#pragma once

#include <algorithm>
#include <cmath>

#include "guppy_msgs/action/navigate.hpp"
#include "guppy_msgs/msg/corner_detection.hpp"

#include <behaviortree_ros2/bt_action_node.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

class MoveTowardBehavior : public BT::RosActionNode<guppy_msgs::action::Navigate> {
 public:
  MoveTowardBehavior(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params) : BT::RosActionNode<guppy_msgs::action::Navigate>(name, conf, params) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts({BT::InputPort<guppy_msgs::msg::CornerDetection>("detection"), BT::InputPort<double>("timeout"), BT::InputPort<bool>("continueOnTimeout")});
  }

  bool setGoal(BT::RosActionNode<guppy_msgs::action::Navigate>::Goal& goal) override {
    RCLCPP_INFO(logger(), "made it to move toward start");
    guppy_msgs::msg::CornerDetection detection;
    getInput("detection", detection);

    if (detection.corners.size() < 2)
      return false;

    double diagonal_size = 0.0;
    for (int i = 0; i < detection.corners.size(); i++) {
      for (int j = i + 1; j < detection.corners.size(); j++) {
        auto corner1 = detection.corners[i];
        auto corner2 = detection.corners[j];
        diagonal_size = std::max(sqrt(pow(corner2.x - corner1.x, 2) + pow(corner2.y - corner1.y, 2)), diagonal_size);
      }
    }
    if (diagonal_size == 0.0)
      return false;

    constexpr double maintain_distance = 0.25;  // arbitrary value
    constexpr double scale = 0.1;               // scaling from pixel units to meters
    constexpr double fov_per_pixel = 0.08;      // made up number, not calculated from anything

    double angle = fov_per_pixel * diagonal_size / 2;
    double distance = (diagonal_size / 2) / tan(angle / 180 * M_PI) * scale;

    goal.pose.position.y = distance > maintain_distance ? distance - maintain_distance : 0.0;
    // then set everything else to zero?
    goal.pose.position.x = goal.pose.position.z = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    Eigen::Quaterniond q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    goal.pose.orientation.w = q.w(), goal.pose.orientation.x = q.x(), goal.pose.orientation.y = q.y(), goal.pose.orientation.z = q.z();

    goal.local = true;  // local to cameras so has to be local

    getInput("timeout", goal.timeout);

    RCLCPP_INFO(logger(), "passed move");

    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult& wrapped) override {
    // should do?
    return BT::NodeStatus::SUCCESS;
  }

  virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override {
    bool continueOnTimeout = false;
    getInput("continueOnTimeout", continueOnTimeout);
    if (continueOnTimeout) {
      RCLCPP_INFO(logger(), "pose setter action aborted, continuing...");
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(logger(), "pose setter node error... %s", BT::toStr(error));
      return BT::NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) { return BT::NodeStatus::RUNNING; }
};
