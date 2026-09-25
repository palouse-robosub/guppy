#include <memory>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>

#include "guppy_msgs/msg/can_frame.hpp"
#include "guppy_util/quality.hpp"

using namespace std::chrono_literals;

class BarometerPublisher : public rclcpp::Node {
private:
    const std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>> barometer_pub_{
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/barometer", quality::volatile_profile)
    };
    std::shared_ptr<rclcpp::Subscription<guppy_msgs::msg::CanFrame>>                  can_sub_;
    double                                                                            offset_ = 0;
    bool                                                                              initialized_ = false;
public:
    BarometerPublisher() : Node("barometer_publisher") {
        this->declare_parameter("tf_frame", "barometer");
        this->can_sub_ = this->create_subscription<guppy_msgs::msg::CanFrame>(
            "/can/id_0x26", quality::volatile_profile,
            [this](const guppy_msgs::msg::CanFrame& msg) {
                can_callback(msg);
            }
        );

        RCLCPP_INFO(this->get_logger(), "setup publisher and subscriber");
    }
private:
    void can_callback(const guppy_msgs::msg::CanFrame& msg) {
        float depth = 0;
        memcpy(&depth, msg.data.data(), sizeof(float));
        if (!this->initialized_) {
            this->offset_      = depth;
            this->initialized_ = true;
        }
        depth -= this->offset_;
        depth *= -1;

        geometry_msgs::msg::PoseWithCovarianceStamped pose_out;
        pose_out.header.frame_id = this->get_parameter("tf_frame").as_string();
        pose_out.header.stamp    = this->get_clock()->now();
        pose_out.pose.pose.position.z = depth;

        // this is chatgpt... need to find actual values
        pose_out.pose.covariance = {
            -1, 0, 0, 0,  0, 0, 0, -1, 0, 0, 0,  0, 0, 0, 0.5, 0, 0, 0,
            0,  0, 0, -1, 0, 0, 0, 0,  0, 0, -1, 0, 0, 0, 0,   0, 0, -1
        };

        barometer_pub_->publish(pose_out);

        RCLCPP_DEBUG(this->get_logger(), "sent pose with altitude %f", depth);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    const auto barometer_node = std::make_shared<BarometerPublisher>();
    rclcpp::spin(barometer_node);
    rclcpp::shutdown();
    return 0;
}
