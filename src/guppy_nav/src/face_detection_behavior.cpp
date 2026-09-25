#include "guppy_nav/face_detection_behavior.hpp"

// public methods
FaceDetectionBehavior::FaceDetectionBehavior(
    const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& parameters
) : BT::RosActionNode<guppy_msgs::action::Navigate>(name, config, parameters) { }

BT::PortsList FaceDetectionBehavior::providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<double>("detection"), BT::InputPort<double>("timeout"),
         BT::InputPort<bool>("continueOnTimeout")}
    );
}

bool FaceDetectionBehavior::setGoal(BT::RosActionNode<guppy_msgs::action::Navigate>::Goal& goal) {
    guppy_msgs::msg::CornerDetection detection;
    this->getInput("detection", detection);

    auto x = 0.0, y = 0.0;
    for (auto corner : detection.corners)
        x += corner.x, y += corner.y;

    auto size  = detection.corners.size();
    x /= size, y /= size;

    auto roll  = 0.0,
         pitch = FaceDetectionBehavior::camera_max_angle_pitch
               * (y / FaceDetectionBehavior::camera_resolution_y),
         yaw = FaceDetectionBehavior::camera_max_angle_yaw
             * (x / FaceDetectionBehavior::camera_resolution_x);
    Eigen::Quaterniond q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                         * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    goal.pose.position.x = goal.pose.position.y = goal.pose.position.z =
        0.0;    // doesn't move position
    goal.pose.orientation.w = q.w(), goal.pose.orientation.x = q.x(),
    goal.pose.orientation.y = q.y(), goal.pose.orientation.z = q.z();

    goal.local = true;    // local to cameras so has to be local

    this->getInput("timeout", goal.timeout);
    return true;
}

BT::NodeStatus FaceDetectionBehavior::onResultReceived(const WrappedResult& wrapped) {
    // should do?
    return BT::NodeStatus::SUCCESS;
    static_cast<void>(wrapped);
}

BT::NodeStatus FaceDetectionBehavior::onFailure(BT::ActionNodeErrorCode error) {
    bool continueOnTimeout = false;
    this->getInput("continueOnTimeout", continueOnTimeout);
    if (continueOnTimeout) {
        RCLCPP_INFO(this->logger(), "pose setter action aborted, continuing...");
        return BT::NodeStatus::SUCCESS;
    } else {
        RCLCPP_ERROR(this->logger(), "pose setter node error... %s", BT::toStr(error));
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus FaceDetectionBehavior::onFeedback(const std::shared_ptr<const Feedback> feedback) {
    return BT::NodeStatus::RUNNING;
    static_cast<void>(feedback);
}
