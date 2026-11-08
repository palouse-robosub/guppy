#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <chrono>
#include <image_transport/publisher.hpp>
#include <memory>
#include <rclcpp/node.hpp>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class USBCameraPublisher : public rclcpp::Node {
  public:
    static std::shared_ptr<USBCameraPublisher> create() {
        auto node = std::make_shared<USBCameraPublisher>();
        node->post_init();
        return node;
    }

    USBCameraPublisher() : Node("usb_camera_publisher") {
        this->declare_parameter("camera_index", 0);
        this->declare_parameter("camera_name", "default");
        this->declare_parameter<std::vector<double>>(
            "camera_model_d", std::vector<double>(5, 0.0)
        );
        this->declare_parameter<std::vector<double>>(
            "camera_model_k", std::vector<double>(9, 0.0)
        );

        int camera_index = this->get_parameter("camera_index").as_int();
        std::string camera_name = this->get_parameter("camera_name").as_string();

        RCLCPP_INFO(this->get_logger(), "Loading camera_index=%d", camera_index);

        camera.open(camera_index);

        info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            std::string("/cam/") + camera_name + "/camera_info", 1
        );
        timer_ = this->create_wall_timer(
            50ms, std::bind(&USBCameraPublisher::timer_callback, this)
        );
    }

    ~USBCameraPublisher() {
        // ...
    }

  private:
    void post_init() {
        it_ = std::make_unique<image_transport::ImageTransport>(
            shared_from_this()
        );
        std::string camera_name = this->get_parameter("camera_name").as_string();
        image_pub_        = it_->advertise(
            "/cam/" + camera_name, rclcpp::SensorDataQoS().get_rmw_qos_profile()
        );
    }

    void timer_callback() {
        camera.grab();
        cv::Mat image;
        camera.retrieve(image);

        rclcpp::Time          now = this->get_clock()->now();
        std_msgs::msg::Header header;
        header.frame_id = this->get_parameter("camera_name").as_string();
        header.stamp    = now;

        msg_ = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
        image_pub_.publish(*msg_.get());

        sensor_msgs::msg::CameraInfo info;
        info.height           = image.rows;
        info.width            = image.cols;
        info.distortion_model = "plumb_bob"; // ...I think... --cole


        auto k_vector = this->get_parameter("camera_model_k").as_double_array();
        if (k_vector.size() == 9) {
            std::copy(k_vector.begin(), k_vector.end(), info.k.begin());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'camera_model_k' must have exactly 9 elements.");
        }


        info.d = this->get_parameter("camera_model_d").as_double_array();

        info_pub_->publish(info);

    }

    cv::VideoCapture camera;

    std::unique_ptr<image_transport::ImageTransport>           it_;
    image_transport::Publisher                                 image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    sensor_msgs::msg::Image::SharedPtr                         msg_;
    rclcpp::TimerBase::SharedPtr                               timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = USBCameraPublisher::create();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
