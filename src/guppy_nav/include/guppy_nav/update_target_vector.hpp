#pragma once

#include "guppy_msgs/msg/transform_list.hpp"
#include <Eigen/Geometry>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <optional>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class UpdateTargetVector :
  public BT::RosTopicSubNode<guppy_msgs::msg::TransformList> {
  public:
    UpdateTargetVector(
        const std::string& name, const BT::NodeConfig& config,
        const BT::RosNodeParams& params
    ) :
      BT::RosTopicSubNode<guppy_msgs::msg::TransformList>(name, config, params),
      node_(params.nh.lock()) {
        if (!node_)
            throw std::runtime_error("UpdateTargetVector: node is null");
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    static BT::PortsList providedPorts() {
        return providedBasicPorts(
            {
              BT::InputPort<std::string>("target", "", "Target name"),
              BT::InputPort<std::string>(
                  "camera_frame", "camera_optical_frame", "Camera frame"
              ),
              BT::InputPort<std::string>("world_frame", "odom", "World frame"),
              BT::InputPort<std::string>(
                  "base_frame", "base_link", "Robot base frame"
              ),
              BT::InputPort<double>(
                  "gate_radius", 1.0, "Max (in meters) before rejecting"
              ),
              BT::InputPort<double>("alpha", 0.3, "Smoothing factor (0-1)"),
              BT::InputPort<double>(
                  "timeout", 5.0, "Seconds before considered lost"
              ),
              BT::InputPort<int>(
                  "max_rejects", 5, "Consecutive rejects before re-seeding"
              ),
              BT::OutputPort<geometry_msgs::msg::Pose>("target_pose"),
            }
        );
    }

    BT::NodeStatus onTick(
        const std::shared_ptr<guppy_msgs::msg::TransformList>& msg
    ) override {
        std::string target, camera_frame, world_frame, base_frame;
        double      gate_radius = 1.0, alpha = 0.3, timeout = 5.0;
        int         max_rejects = 5;
        getInput("target", target);
        getInput("camera_frame", camera_frame);
        getInput("world_frame", world_frame);
        getInput("base_frame", base_frame);
        getInput("gate_radius", gate_radius);
        getInput("alpha", alpha);
        getInput("timeout", timeout);
        getInput("max_rejects", max_rejects);
        const auto now = node_->get_clock()->now();

        // 1) Processes a new message in the WORLD frame.
        if (msg) {
            for (const auto& t : msg->transforms) {
                if (target.empty() || t.child_frame_id == target) {
                    if (processMeasurement(
                            t, camera_frame, world_frame, gate_radius, alpha,
                            max_rejects
                        )) {
                        last_seen_ = now;
                    }
                    break;
                }
            }
        }

        // 2) Timeout window
        if (!last_seen_ || !ema_w_ || (now - *last_seen_).seconds() > timeout) {
            RCLCPP_WARN_THROTTLE(
                logger(), *node_->get_clock(), 2000,
                "UpdateTargetVector: target not seen or stale."
            );
            return BT::NodeStatus::FAILURE;
        }

        // 3. Outputs current target pose in BASE frame.
        try {
            auto tf_w_b = tf_buffer_->lookupTransform(
                world_frame, base_frame, tf2::TimePointZero
            );
            Eigen::Isometry3d T_w_b = tf2::transformToEigen(tf_w_b.transform);

            // Base -> Target = (World -> Base)^-1 * (World -> Target)
            Eigen::Isometry3d T_b_t = T_w_b.inverse() * (*ema_w_);

            geometry_msgs::msg::Pose out;
            out.position.x = T_b_t.translation().x();
            out.position.y = T_b_t.translation().y();
            out.position.z = T_b_t.translation().z();

            Eigen::Quaterniond q(T_b_t.linear());
            q.normalize();
            out.orientation.w = q.w();
            out.orientation.x = q.x();
            out.orientation.y = q.y();
            out.orientation.z = q.z();

            setOutput("target_pose", out);

            // Print info
            RCLCPP_INFO(
                logger(), "UpdateTargetVector: target at (%.2f, %.2f, %.2f)",
                out.position.x, out.position.y, out.position.z
            );

            return BT::NodeStatus::SUCCESS;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(
                logger(), "UpdateTargetVector: TF lookup failed: %s", ex.what()
            );
            return BT::NodeStatus::FAILURE;
        }
    }

  private:
    bool processMeasurement(
        const geometry_msgs::msg::TransformStamped& tag_cam,
        const std::string& camera_frame, const std::string& world_frame,
        double gate_radius, double alpha, int max_rejects
    ) {
        const auto&  r  = tag_cam.transform.rotation;
        const auto&  tr = tag_cam.transform.translation;
        const double qn =
            std::sqrt(r.x * r.x + r.y * r.y + r.z * r.z + r.w * r.w);
        if (!std::isfinite(qn) || qn < 0.1 || !std::isfinite(tr.x)
            || !std::isfinite(tr.y) || !std::isfinite(tr.z)) {
            RCLCPP_WARN(
                logger(), "UpdateTargetVector: invalid detection, skipping."
            );
            return false;
        }
        try {
            const auto stamp = tag_cam.header.stamp;
            auto       tf_w_c =
                tf_buffer_->lookupTransform(world_frame, camera_frame, stamp);

            Eigen::Isometry3d T_w_c = tf2::transformToEigen(tf_w_c.transform);
            Eigen::Isometry3d T_c_t = tf2::transformToEigen(tag_cam.transform);

            const Eigen::Isometry3d T_w_t = T_w_c * T_c_t;

            if (!ema_w_) {
                ema_w_               = T_w_t;
                consecutive_rejects_ = 0;
                RCLCPP_INFO(
                    logger(), "UpdateTargetVector: seeded target in world."
                );
                return true;
            }

            const double dist =
                (T_w_t.translation() - ema_w_->translation()).norm();
            if (dist < gate_radius) {
                ema_w_->translation() = alpha * T_w_t.translation()
                                      + (1.0 - alpha) * ema_w_->translation();

                Eigen::Quaterniond q_curr(T_w_t.linear());
                Eigen::Quaterniond q_ema(ema_w_->linear());

                q_curr.normalize();
                q_ema.normalize();

                // Ensures on taking the shortest path
                if (q_ema.dot(q_curr) < 0.0)
                    q_curr.coeffs() *= -1.0;

                ema_w_->linear() =
                    q_ema.slerp(alpha, q_curr).toRotationMatrix();
                consecutive_rejects_ = 0;
                return true;
            }

            ++consecutive_rejects_;
            if (consecutive_rejects_ >= max_rejects) {
                RCLCPP_WARN(
                    logger(),
                    "UpdateTargetVector: %d consecutive rejects "
                    "(dist %.2fm); re-seeding filter.",
                    consecutive_rejects_, dist
                );
                ema_w_               = T_w_t;
                consecutive_rejects_ = 0;
                return true;
            }

            RCLCPP_WARN_THROTTLE(
                logger(), *node_->get_clock(), 1000,
                "UpdateTargetVector: rejected outlier (%.2fm). Keeping old.",
                dist
            );
            return false;

        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(
                logger(), "UpdateTargetVector: tf lookup failed: %s", ex.what()
            );
            return false;
        }
    }

    rclcpp::Node::SharedPtr                     node_;
    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::optional<Eigen::Isometry3d>            ema_w_;
    std::optional<rclcpp::Time>                 last_seen_;
    int                                         consecutive_rejects_ = 0;
};
