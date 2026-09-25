#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>

#include "guppy_util/quality.hpp"

class CombineSensors : public rclcpp::Node {

private:
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>>                          odom_pub_ =
        this->create_publisher<nav_msgs::msg::Odometry>(
            "/odometry/filtered", quality::volatile_profile
        );
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>>                         imu_sub_ =
        this->create_subscription<sensor_msgs::msg::Imu>(
            "/vectornav/imu", quality::volatile_profile,
            [this](const std::shared_ptr<const sensor_msgs::msg::Imu>& msg) {
                this->imu_callback(*msg);
            }
        );
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>>                       dvl_sub_ =
        this->create_subscription<nav_msgs::msg::Odometry>(
            "/waterlinked_dvl_driver/odom", quality::volatile_profile,
            [this](const std::shared_ptr<const nav_msgs::msg::Odometry>& msg) {
                this->dvl_callback(*msg);
            }
        );
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>> baro_sub_ =
        this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/barometer", quality::volatile_profile,
            [this](const std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped>& msg) {
                this->baro_callback(*msg);
            }
        );
    std::unique_ptr<tf2_ros::TransformBroadcaster>                                       tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      nav_msgs::msg::Odometry odom_;

      Eigen::Quaterniond        initial_orientation_;
      geometry_msgs::msg::Point initial_pose_{};
      bool                      has_initial_ = false;

      std::shared_ptr<rclcpp::Service<std_srvs::srv::Empty>> reset_service_;
  public:
    CombineSensors() : Node("combine_sensors") {
        reset_service_ = this->create_service<std_srvs::srv::Empty>(
            "reset_odom",
            [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
                std::shared_ptr<std_srvs::srv::Empty::Response>) {
                initial_pose_.x      = odom_.pose.pose.position.x;
                initial_pose_.y      = odom_.pose.pose.position.y;
                initial_pose_.z      = odom_.pose.pose.position.z;
                initial_orientation_ = Eigen::Quaterniond(
                    odom_.pose.pose.orientation.w, odom_.pose.pose.orientation.x,
                    odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z
                );
            },
            quality::volatile_profile
        );

        geometry_msgs::msg::TransformStamped msg;
        msg.header.frame_id = "map";
        msg.header.stamp    = this->get_clock()->now();
        msg.child_frame_id  = "odom";
        tf_broadcaster_->sendTransform(msg);
    }


    ~CombineSensors() {
        // ...
    }

    void baro_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped& msg
    ) {
        odom_.pose.pose.position.z = msg.pose.pose.position.z - initial_pose_.z;
        odom_pub_->publish(odom_);
        publish_transform();
    }

    void dvl_callback(const nav_msgs::msg::Odometry& msg) {
        // Rotate DVL frame -> body frame (your existing transform)
        Eigen::Quaterniond q_dvl_to_body(
            Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY())
        );

        Eigen::Vector3d position(
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            msg.pose.pose.position.z
        );

        Eigen::Vector3d twist(
            msg.twist.twist.linear.x, msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        );

        position = q_dvl_to_body * position;
        twist    = q_dvl_to_body * twist;

        // DVL location relative to vehicle origin (body frame)
        Eigen::Vector3d r_body(0.2, 0.0, -0.125);

        // Current vehicle orientation (replace with your IMU orientation)
        Eigen::Quaterniond q_world_body(
            odom_.pose.pose.orientation.w, odom_.pose.pose.orientation.x,
            odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z
        );

        // Rotate lever arm into world frame
        Eigen::Vector3d r_world = q_world_body * r_body;

        // Correct position to vehicle origin
        Eigen::Vector3d corrected_position = position - r_world;

        odom_.pose.pose.position.x = corrected_position.x() - initial_pose_.x;
        odom_.pose.pose.position.y = corrected_position.y() - initial_pose_.y;
        odom_.pose.pose.position.z = corrected_position.z() - initial_pose_.z;

        // Lever arm velocity correction
        Eigen::Vector3d omega(
            odom_.twist.twist.angular.x, odom_.twist.twist.angular.y,
            odom_.twist.twist.angular.z
        );

        Eigen::Vector3d corrected_twist = twist - omega.cross(r_body);

        odom_.twist.twist.linear.x = corrected_twist.x();
        odom_.twist.twist.linear.y = corrected_twist.y();
        odom_.twist.twist.linear.z = corrected_twist.z();

        odom_pub_->publish(odom_);
        publish_transform();
    }

    void imu_callback(const sensor_msgs::msg::Imu& msg) {
        Eigen::Vector3d    axis_vector(0.0, 0.0, 1.0);
        Eigen::AngleAxisd  angle_axis(M_PI * -0.5, axis_vector);
        Eigen::Quaterniond quat(angle_axis);

        Eigen::Quaternion orientation(
            msg.orientation.w, msg.orientation.x, msg.orientation.y,
            msg.orientation.z
        );
        Eigen::Vector3d twist(
            msg.angular_velocity.x, msg.angular_velocity.y,
            msg.angular_velocity.z
        );

        orientation = orientation * quat;
        twist       = quat * twist;

        if (!this->has_initial_) {
            this->initial_orientation_ = orientation;
            this->has_initial_         = true;
        }

        orientation *= this->initial_orientation_.inverse();

        odom_.pose.pose.orientation.w = orientation.w();
        odom_.pose.pose.orientation.x = orientation.x();
        odom_.pose.pose.orientation.y = orientation.y();
        odom_.pose.pose.orientation.z = orientation.z();
        odom_.twist.twist.angular.x   = twist[0];
        odom_.twist.twist.angular.y   = twist[1];
        odom_.twist.twist.angular.z   = twist[2];
        odom_pub_->publish(odom_);

        publish_transform();
    }

    void publish_transform() {
        geometry_msgs::msg::TransformStamped msg;
        msg.header.frame_id = "odom";
        msg.header.stamp    = this->get_clock()->now();
        msg.child_frame_id  = "dvl_link";

        msg.transform.rotation.w    = odom_.pose.pose.orientation.w;
        msg.transform.rotation.x    = odom_.pose.pose.orientation.x;
        msg.transform.rotation.y    = odom_.pose.pose.orientation.y;
        msg.transform.rotation.z    = odom_.pose.pose.orientation.z;
        msg.transform.translation.x = odom_.pose.pose.position.x;
        msg.transform.translation.y = odom_.pose.pose.position.y;
        msg.transform.translation.z = odom_.pose.pose.position.z;

        // tf_broadcaster_->sendTransform(msg);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    const auto combiner = std::make_shared<CombineSensors>();
    rclcpp::spin(combiner);
    rclcpp::shutdown();
    return 0;
}
