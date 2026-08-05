#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "std_srvs/srv/empty.hpp"



#include <Eigen/Geometry>

class CombineSensors : public rclcpp::Node {
public:
  CombineSensors() : Node("combine_sensors") {
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("/vectornav/imu",10,std::bind(&CombineSensors::imu_callback, this, std::placeholders::_1));
    dvl_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/waterlinked_dvl_driver/odom",10,std::bind(&CombineSensors::dvl_callback, this, std::placeholders::_1));
    baro_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/barometer",10,std::bind(&CombineSensors::baro_callback, this, std::placeholders::_1));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    initial_pose.x = 0.0;
    initial_pose.y = 0.0;
    initial_pose.z = 0.0;

    reset_service = this->create_service<std_srvs::srv::Empty>("reset_odom", [&](const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response) {
      initial_pose.x = odom.pose.pose.position.x;
      initial_pose.y = odom.pose.pose.position.y;
      initial_pose.z = odom.pose.pose.position.z;
      initial_orientation = Eigen::Quaterniond(
        odom.pose.pose.orientation.w,
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z
      );
    }, 10);

    geometry_msgs::msg::TransformStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = this->get_clock()->now();
    msg.child_frame_id = "odom";
    tf_broadcaster_->sendTransform(msg);
  }

  ~CombineSensors() {
    // ...
  }

void baro_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    odom.pose.pose.position.z = msg->pose.pose.position.z - initial_pose.z;

    odom_pub_->publish(odom);

    publish_transform();    
  }

  void dvl_callback(nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Rotate DVL frame -> body frame (your existing transform)
    Eigen::Quaterniond q_dvl_to_body(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()));

    Eigen::Vector3d position(msg->pose.pose.position.x,
                             msg->pose.pose.position.y,
                             msg->pose.pose.position.z);

    Eigen::Vector3d twist(msg->twist.twist.linear.x,
                          msg->twist.twist.linear.y,
                          msg->twist.twist.linear.z);

    position = q_dvl_to_body * position;
    twist    = q_dvl_to_body * twist;

    // DVL location relative to vehicle origin (body frame)
    Eigen::Vector3d r_body(0.2, 0.0, -0.125);

    // Current vehicle orientation (replace with your IMU orientation)
    Eigen::Quaterniond q_world_body(
        odom.pose.pose.orientation.w,
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z);

    // Rotate lever arm into world frame
    Eigen::Vector3d r_world = q_world_body * r_body;

    // Correct position to vehicle origin
    Eigen::Vector3d corrected_position = position - r_world;

    odom.pose.pose.position.x = corrected_position.x() - initial_pose.x;
    odom.pose.pose.position.y = corrected_position.y() - initial_pose.y;
    odom.pose.pose.position.z = corrected_position.z() - initial_pose.z;

    // Lever arm velocity correction
    Eigen::Vector3d omega(
        odom.twist.twist.angular.x,
        odom.twist.twist.angular.y,
        odom.twist.twist.angular.z);

    Eigen::Vector3d corrected_twist = twist - omega.cross(r_body);

    odom.twist.twist.linear.x = corrected_twist.x();
    odom.twist.twist.linear.y = corrected_twist.y();
    odom.twist.twist.linear.z = corrected_twist.z();

    odom_pub_->publish(odom);
    publish_transform();
}

  void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg) {
    Eigen::Vector3d axis_vector(0.0, 0.0, 1.0);
    Eigen::AngleAxisd angle_axis(M_PI * -0.5, axis_vector);
    Eigen::Quaterniond quat(angle_axis);

    Eigen::Quaternion orientation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    Eigen::Vector3d twist(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    orientation = orientation * quat;
    twist = quat * twist;

    if (!this->has_initial) {
      this->initial_orientation = orientation;
      this->has_initial = true;
    }

    orientation *= this->initial_orientation.inverse();

    odom.pose.pose.orientation.w = orientation.w();
    odom.pose.pose.orientation.x = orientation.x();
    odom.pose.pose.orientation.y = orientation.y();
    odom.pose.pose.orientation.z = orientation.z();
    odom.twist.twist.angular.x = twist[0];
    odom.twist.twist.angular.y = twist[1];
    odom.twist.twist.angular.z = twist[2];
    odom_pub_->publish(odom);

    publish_transform();    
  }

  void publish_transform() {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.frame_id = "odom";
    msg.header.stamp = this->get_clock()->now();
    msg.child_frame_id = "dvl_link";

    msg.transform.rotation.w = odom.pose.pose.orientation.w;
    msg.transform.rotation.x = odom.pose.pose.orientation.x;
    msg.transform.rotation.y = odom.pose.pose.orientation.y;
    msg.transform.rotation.z = odom.pose.pose.orientation.z;
    msg.transform.translation.x = odom.pose.pose.position.x;
    msg.transform.translation.y = odom.pose.pose.position.y;
    msg.transform.translation.z = odom.pose.pose.position.z;

    // tf_broadcaster_->sendTransform(msg);
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr dvl_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr baro_subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  nav_msgs::msg::Odometry odom;

  Eigen::Quaterniond initial_orientation;
  geometry_msgs::msg::Point initial_pose;
  bool has_initial = false;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service;

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<CombineSensors>());

  rclcpp::shutdown();

  return 0;
}