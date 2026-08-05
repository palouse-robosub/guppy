#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <guppy_msgs/msg/state.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <guppy_nav/change_state_behavior.hpp>
#include <guppy_nav/pose_setter_behavior.hpp>
#include <guppy_nav/acquire_detection.hpp>
#include <guppy_nav/face_detection_behavior.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "guppy_msgs/msg/state.hpp"
#include "guppy_nav/face_detection_behavior.hpp"

#define TICK_MS 20
#define TREE_NAME "practice-run"

class NavigationBehaviorTree : public rclcpp::Node {
public:
    NavigationBehaviorTree() : Node("navigation_behavior_tree") {
        BT::BehaviorTreeFactory factory;

        _change_state_client = std::make_shared<rclcpp::Node>("chage_state_behavior_client");
        _navigation_client = std::make_shared<rclcpp::Node>("navigate_behavior_client");
        _detection_subscriber = std::make_shared<rclcpp::Node>("detection_subscriber");

        BT::RosNodeParams stateParameters(_change_state_client, "change_state");
        BT::RosNodeParams navigateParameters(_navigation_client, "/navigate");
        BT::RosNodeParams detectionParameters(_detection_subscriber, "/cam/test/detections");

        factory.registerNodeType<ChangeStateBehavior>("ChangeState", stateParameters);
        factory.registerNodeType<NavigateBehavior>("Navigate", navigateParameters);
        factory.registerNodeType<FaceDetectionBehavior>("FaceDetection", navigateParameters);
        factory.registerNodeType<AcquireDetection>("AcquireDetection", detectionParameters);

        _tree = std::make_unique<BT::Tree>(factory.createTreeFromFile("./src/guppy_tasks/resource/" + std::string(TREE_NAME) + ".xml"));

        auto tick = [this]() {
            if (!_running) return;
            _tree->tickOnce();
        };

        auto onState = [this](guppy_msgs::msg::State::UniquePtr msg) {
            auto nav = msg->state == guppy_msgs::msg::State::NAV;
            if (nav && !_running) _running = true;
            else if (!nav && _running) {
                _running = false;
                _tree->haltTree();
            }
        };

        _timer = this->create_wall_timer(std::chrono::milliseconds(TICK_MS), tick);

        auto state_quality = rclcpp::QoS(rclcpp::KeepLast(1)).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        _subscription = this->create_subscription<guppy_msgs::msg::State>("state", state_quality, onState);

        RCLCPP_INFO(get_logger(), "Behavior tree %s initialized with %lu registered nodes.", TREE_NAME, factory.builders().size());
    }
private:
    std::unique_ptr<BT::Tree> _tree;

    std::shared_ptr<rclcpp::Node> _change_state_client;
    std::shared_ptr<rclcpp::Node> _navigation_client;
    std::shared_ptr<rclcpp::Node> _detection_subscriber;

    rclcpp::Subscription<guppy_msgs::msg::State>::SharedPtr _subscription;
    rclcpp::TimerBase::SharedPtr _timer;

    bool _running = false;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationBehaviorTree>());

    rclcpp::shutdown();

    return 0;
}
