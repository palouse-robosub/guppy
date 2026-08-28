#pragma once

#include "guppy_msgs/action/navigate.hpp"
#include "guppy_msgs/msg/corner_detection.hpp"

#include <behaviortree_ros2/bt_action_node.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#define CAMERA_RES_X 1920
#define CAMERA_RES_Y 1080

#define CAMERA_MAX_ANGLE_YAW 1.57079632679
#define CAMERA_MAX_ANGLE_PITCH 1.57079632679

// https://www.desmos.com/calculator/ivz6gpks8n

class FaceDetectionBehavior
    : public BT::RosActionNode<guppy_msgs::action::Navigate> {
 public:
  FaceDetectionBehavior(const std::string& name,
                        const BT::NodeConfig& conf,
                        const BT::RosNodeParams& params)
      : BT::RosActionNode<guppy_msgs::action::Navigate>(name, conf, params) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts({BT::InputPort<double>("detection"),
                               BT::InputPort<double>("timeout"),
                               BT::InputPort<bool>("continueOnTimeout")});
  }

  bool setGoal(
      BT::RosActionNode<guppy_msgs::action::Navigate>::Goal& goal) override {
    guppy_msgs::msg::CornerDetection detection;
    getInput("detection", detection);

    auto x = 0.0, y = 0.0;
    for (auto corner : detection.corners)
      x += corner.x, y += corner.y;

    auto size = detection.corners.size();
    x /= size, y /= size;

    auto roll = 0.0, pitch = CAMERA_MAX_ANGLE_PITCH * (y / CAMERA_RES_Y),
         yaw = CAMERA_MAX_ANGLE_YAW * (x / CAMERA_RES_X);
    Eigen::Quaterniond q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    goal.pose.position.x = goal.pose.position.y = goal.pose.position.z =
        0.0;  // doesn't move position
    goal.pose.orientation.w = q.w(), goal.pose.orientation.x = q.x(),
    goal.pose.orientation.y = q.y(), goal.pose.orientation.z = q.z();

    goal.local = true;  // local to cameras so has to be local

    getInput("timeout", goal.timeout);
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

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) {
    return BT::NodeStatus::RUNNING;
  }
};
