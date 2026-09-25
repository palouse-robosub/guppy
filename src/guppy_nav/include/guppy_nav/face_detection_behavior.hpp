#ifndef FACE_DETECTION_BEHAVIOR_HPP
#define FACE_DETECTION_BEHAVIOR_HPP

#include "guppy_msgs/action/navigate.hpp"
#include "guppy_msgs/msg/corner_detection.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <behaviortree_ros2/bt_action_node.hpp>

// https://www.desmos.com/calculator/ivz6gpks8n

class FaceDetectionBehavior : public BT::RosActionNode<guppy_msgs::action::Navigate> {
private:
    static constexpr const inline auto camera_resolution_x    = 1920U;
    static constexpr const inline auto camera_resolution_y    = 1080U;
    static constexpr const inline auto camera_max_angle_yaw   = 1.57079632679;
    static constexpr const inline auto camera_max_angle_pitch = 1.57079632679;
public:
    FaceDetectionBehavior(
        const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params
    );
    static BT::PortsList providedPorts();
    bool           setGoal(BT::RosActionNode<guppy_msgs::action::Navigate>::Goal& goal) override;
    BT::NodeStatus onResultReceived(const WrappedResult& wrapped) override;
    virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
    BT::NodeStatus         onFeedback(const std::shared_ptr<const Feedback> feedback) override;
};

#endif
