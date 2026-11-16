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
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <chrono>
#include <map>
#include <string>
#include <vector>

#define SLAM_ITERATIONS_NEEDED 100

using namespace std::chrono_literals;

class SlamNode : public rclcpp::Node {
  public:
    SlamNode() : Node("slam") {
        this->declare_parameter("camera_name", "default_camera");

        std::string camera_name =
            this->get_parameter("camera_name").as_string();

        output_pub_ = this->create_subscription<guppy_msgs::msg::TransformList>(
            std::string("/cam/") + camera_name + "/transforms", 10,
            std::bind(&SlamNode::callback, this, std::placeholders::_1)
        );

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_static_broadcaster_ =
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

        
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "odom";

        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
        // tf_static_broadcaster_->sendTransform(t);
    }

    ~SlamNode() {
        // ...
    }

    // function from AI
    tf2::Transform averageTransforms(const std::vector<tf2::Transform>& ts) {
        tf2::Vector3    p(0, 0, 0);
        Eigen::Matrix4d A = Eigen::Matrix4d::Zero();

        for (const auto& t : ts) {
            p += t.getOrigin();

            auto            q = t.getRotation();
            Eigen::Vector4d v(q.x(), q.y(), q.z(), q.w());
            A += v * v.transpose();
        }

        p /= ts.size();

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(A);
        auto q = es.eigenvectors().col(3);

        return tf2::Transform(
            tf2::Quaternion(q[0], q[1], q[2], q[3]).normalized(), p
        );
    }

    void callback(guppy_msgs::msg::TransformList::UniquePtr msg) {
        // std::vector<tf2::Transform> potential_odoms;
        geometry_msgs::msg::TransformStamped tf_msg_map_odom =
        tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);

        tf2::Transform tf_map_odom;
        tf2::fromMsg(tf_msg_map_odom.transform, tf_map_odom);

        for (geometry_msgs::msg::TransformStamped tf_msg_cam_obj :
             msg->transforms) {
            std::string object_name = tf_msg_cam_obj.child_frame_id;
            std::string camera_name = tf_msg_cam_obj.header.frame_id;

            if (historical_transforms.count(object_name) > 0
                && historical_transforms[object_name].size()
                       == SLAM_ITERATIONS_NEEDED) {
                tf2::Transform tf_map_obj =
                    averageTransforms(historical_transforms[object_name]);

                historical_transforms[object_name].push_back(tf_map_obj);

                geometry_msgs::msg::TransformStamped tf_msg_map_obj;

                tf_msg_map_obj.child_frame_id  = object_name;
                tf_msg_map_obj.header.frame_id = "map";
                tf_msg_map_obj.header.stamp    = this->get_clock()->now();
                tf2::toMsg(tf_map_obj, tf_msg_map_obj.transform);

                tf_static_broadcaster_->sendTransform(tf_msg_map_obj);
            }
            else if (historical_transforms[object_name].size() > SLAM_ITERATIONS_NEEDED) {
                builtin_interfaces::msg::Time time = tf_msg_cam_obj.header.stamp;

                geometry_msgs::msg::TransformStamped tf_msg_cam_obj_predicted =
                        tf_buffer_->lookupTransform(camera_name, object_name, tf2::TimePointZero);

                tf2::Transform tf_cam_obj_predicted;
                tf2::fromMsg(tf_msg_cam_obj_predicted.transform, tf_cam_obj_predicted);

                tf2::Transform tf_cam_obj_actual;
                tf2::fromMsg(tf_msg_cam_obj.transform, tf_cam_obj_actual);

                tf2::Transform tf_diff = tf_cam_obj_predicted.inverseTimes(tf_cam_obj_actual);

                RCLCPP_INFO(this->get_logger(), "difference for [%s] -> <x:\t %.2f,y:\t %.2f,z:\t %.2f>", object_name.c_str(), tf_diff.getOrigin().x(), tf_diff.getOrigin().y(), tf_diff.getOrigin().z());

                // tf_map_odom *= tf_diff;
            }
            else {
                builtin_interfaces::msg::Time time =
                    tf_msg_cam_obj.header.stamp;

                try {
                    geometry_msgs::msg::TransformStamped tf_msg_map_cam =
                        tf_buffer_->lookupTransform("map", camera_name, tf2::TimePointZero);

                    tf2::Transform tf_map_cam;
                    tf2::fromMsg(tf_msg_map_cam.transform, tf_map_cam);

                    tf2::Transform tf_cam_obj;
                    tf2::fromMsg(tf_msg_cam_obj.transform, tf_cam_obj);

                    tf2::Transform tf_map_obj = tf_map_cam * tf_cam_obj;

                    historical_transforms[object_name].push_back(tf_map_obj);

                    RCLCPP_INFO(this->get_logger(), "incrementing object %s to %d", object_name.c_str(), historical_transforms[object_name].size());

                    // tf_broadcaster_->sendTransform(tf_msg_cam_obj);
                } catch (const tf2::TransformException& ex) {
                    RCLCPP_WARN(
                        this->get_logger(), "TF lookup failed: %s", ex.what()
                    );
                }
            }

            geometry_msgs::msg::TransformStamped tf_msg_map_odom_new;

            tf_msg_map_odom_new.header.frame_id = "map";
            tf_msg_map_odom_new.child_frame_id  = "odom";
            tf_msg_map_odom_new.header.stamp    = this->get_clock()->now();
            tf2::toMsg(tf_map_odom, tf_msg_map_odom_new.transform);
            tf_broadcaster_->sendTransform(tf_msg_map_odom_new);
        }
    }

  private:
    rclcpp::Subscription<guppy_msgs::msg::TransformList>::SharedPtr output_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>       tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;

    bool has_map_transform = false;
    std::map<std::string, std::vector<tf2::Transform>> historical_transforms;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<SlamNode>());

    rclcpp::shutdown();

    return 0;
}
