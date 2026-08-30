#include "guppy_msgs/msg/transform_list.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <map>
#include <vector>

using namespace std::chrono_literals;

class SlamNode : public rclcpp::Node {
  public:
    SlamNode() : Node("slam") {
        output_pub_ = this->create_subscription<guppy_msgs::msg::TransformList>(
            "/cam/test/transforms", 10,
            std::bind(&SlamNode::callback, this, std::placeholders::_1)
        );

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_static_broadcaster_ =
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    }

    ~SlamNode() {
        // ...
    }

    void callback(guppy_msgs::msg::TransformList::UniquePtr msg) {
        std::vector<tf2::Transform> potential_odoms;

        for (geometry_msgs::msg::TransformStamped tf_msg_cam_object :
             msg->transforms) {
            std::string name = tf_msg_cam_object.child_frame_id;
            tf_broadcaster_->sendTransform(tf_msg_cam_object);

            if (already_seen.count(name) > 0) {
                tf2::Transform tf_map_obj = already_seen[name];
                tf2::Transform tf_odom_object;
                geometry_msgs::msg::TransformStamped tf_msg_odom_object =
                    tf_buffer_->lookupTransform(
                        "odom", name, this->get_clock()->now()
                    );
                tf2::fromMsg(tf_msg_odom_object.transform, tf_odom_object);

                potential_odoms.push_back(
                    tf_map_obj * tf_odom_object.inverse()
                );
            } else {
                geometry_msgs::msg::TransformStamped tf_msg_map_object =
                    tf_buffer_->lookupTransform(
                        "map", name, this->get_clock()->now()
                    );
                tf2::fromMsg(tf_msg_map_object.transform, already_seen[name]);
            }
        }

        if (potential_odoms.size() > 0) {
            geometry_msgs::msg::TransformStamped tfStampedOut;

            tfStampedOut.child_frame_id  = "odom";
            tfStampedOut.header.stamp    = this->get_clock()->now();
            tfStampedOut.header.frame_id = "map";

            tfStampedOut.transform = tf2::toMsg(potential_odoms[0]);
            tf_broadcaster_->sendTransform(tfStampedOut);
        }
    }

  private:
    rclcpp::Subscription<guppy_msgs::msg::TransformList>::SharedPtr output_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>       tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;

    bool                                  has_map_transform = false;
    std::map<std::string, tf2::Transform> already_seen;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<SlamNode>());

    rclcpp::shutdown();

    return 0;
}
