#include <memory>

#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include "behaviortree_cpp/bt_factory.h"

#include <rclcpp/node.hpp>

#include "guppy_msgs/msg/state.hpp"
#include "guppy_nav/acquire_detection.hpp"
#include "guppy_nav/change_state_behavior.hpp"
#include "guppy_nav/face_detection_behavior.hpp"
#include "guppy_nav/pose_setter_behavior.hpp"
#include "guppy_nav/face_detection_behavior.hpp"
#include "guppy_util/quality.hpp"

class NavigationBehaviorTree : public rclcpp::Node {
public:
    static constexpr const inline auto tick_ms = 20U;
private:
    std::unique_ptr<BT::Tree> tree_;
    std::unique_ptr<BT::BehaviorTreeFactory> factory_;

    const std::shared_ptr<rclcpp::Node> change_state_client_ = std::make_shared<rclcpp::Node>("chage_state_behavior_client");
    const std::shared_ptr<rclcpp::Node> navigation_client_ = std::make_shared<rclcpp::Node>("navigate_behavior_client");
    const std::shared_ptr<rclcpp::Node> detection_subscriber_ = std::make_shared<rclcpp::Node>("detection_subscriber");

    rclcpp::Subscription<guppy_msgs::msg::State>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr                            timer_;

    bool running_ = false;
public:
    NavigationBehaviorTree() : Node("navigation_behavior_tree") {
        BT::RosNodeParams state_parameters(this->change_state_client_, "change_state");
        BT::RosNodeParams navigate_parameters(this->navigation_client_, "/navigate");
        BT::RosNodeParams detection_parameters(
            this->detection_subscriber_, "/cam/test/detections"
        );

        this->factory_->registerNodeType<ChangeStateBehavior>(
            "ChangeState", state_parameters
        );
        this->factory_->registerNodeType<NavigateBehavior>(
            "Navigate", navigate_parameters
        );
        this->factory_->registerNodeType<FaceDetectionBehavior>(
            "FaceDetection", navigate_parameters
        );
        this->factory_->registerNodeType<AcquireDetection>(
            "AcquireDetection", detection_parameters
        );

        auto tick = [this]() {
            if (!this->running_)
                return;
            tree_->tickOnce();
        };

        auto on_state = [this](std::unique_ptr<const guppy_msgs::msg::State> msg) {
            auto nav = msg->state == guppy_msgs::msg::State::NAV;
            if (nav && !this->running_)
                this->running_ = true;
            else if (!nav && this->running_) {
                this->running_ = false;
                this->tree_->haltTree();
            }
        };

        timer_ =this->create_wall_timer(std::chrono::milliseconds(NavigationBehaviorTree::tick_ms), tick);

        this->subscription_ = this->create_subscription<guppy_msgs::msg::State>(
            "state", quality::keep_last_profile, on_state
        );
    }
private:
    void initialize_tree() {
        std::string tree_name;
        this->declare_parameter("tree_name", "main");
        this->get_parameter("tree_name", tree_name);

        this->tree_ = std::make_unique<BT::Tree>(
            this->factory_->createTreeFromFile("./src/guppy_tasks/resource/" + tree_name + ".xml")
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Behavior tree %s initialized with %lu registered nodes.",
            tree_name.c_str(), this->factory_->builders().size()
        );
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    const auto behavior_tree_node = std::make_shared<NavigationBehaviorTree>();
    rclcpp::spin(behavior_tree_node);
    rclcpp::shutdown();
    return 0;
}
