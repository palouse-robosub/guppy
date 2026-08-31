#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "guppy_msgs/msg/corner_detection_list.hpp"
#include "guppy_msgs/msg/transform_list.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <vector>

using namespace std::chrono_literals;

class PerspectiveNPoint : public rclcpp::Node {
  public:
    PerspectiveNPoint() : Node("pnp") {
        corner_sub_ =
            this->create_subscription<guppy_msgs::msg::CornerDetectionList>(
                "/cam/test/detections", 10,
                std::bind(
                    &PerspectiveNPoint::callback, this, std::placeholders::_1
                )
            );
        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/cam/test/camera_info", 10,
            std::bind(
                &PerspectiveNPoint::info_callback, this, std::placeholders::_1
            )
        );

        output_pub_ = this->create_publisher<guppy_msgs::msg::TransformList>(
            "/cam/test/transforms", 10
        );
    }

    ~PerspectiveNPoint() {
        // ...
    }

    void callback(guppy_msgs::msg::CornerDetectionList::UniquePtr msg) {
        guppy_msgs::msg::TransformList outputs;
        for (int result_i = 0; result_i < msg->detections.size(); result_i++) {
            auto detection = msg->detections[result_i];

            std::vector<cv::Point3d> objectPoints;
            for (int dimension_i = 0;
                 dimension_i < detection.dimension_points.size();
                 dimension_i++) {
                auto point = detection.dimension_points[dimension_i];
                objectPoints.push_back(cv::Point3d(point.x, point.y, point.z));
            }

            std::vector<cv::Point2d> imagePoints;
            for (int corner_i = 0; corner_i < detection.corners.size();
                 corner_i++) {
                auto point = detection.corners[corner_i];
                imagePoints.push_back(cv::Point2d(point.x, point.y));
            }

            cv::Mat k = cv::Mat(3, 3, CV_64F, camera_info_.k.data()).clone();
            cv::Mat d = cv::Mat(1, 5, CV_64F, camera_info_.d.data()).clone();

            cv::Mat tvec;
            cv::Mat rvec;

            cv::solvePnP(
                objectPoints, imagePoints, k, d, rvec, tvec, 0,
                cv::SolvePnPMethod::SOLVEPNP_IPPE_SQUARE
            );

            RCLCPP_DEBUG(
                this->get_logger(), "%f\t%f\t%f\t%f\t%f\t%f\t\t%s",
                tvec.at<double>(0, 0), tvec.at<double>(1, 0),
                tvec.at<double>(2, 0), rvec.at<double>(0, 0),
                rvec.at<double>(1, 0), rvec.at<double>(2, 0),
                detection.name.c_str()
            );

            double          theta = cv::norm(rvec);
            Eigen::Vector3d axis(
                rvec.at<double>(0, 0), rvec.at<double>(1, 0),
                rvec.at<double>(2, 0)
            );
            axis.normalize();
            Eigen::AngleAxis<double> aa(theta, axis);
            auto                     quat = Eigen::Quaternion<double>(aa);

            geometry_msgs::msg::TransformStamped output;
            output.header                  = msg->header;
            output.child_frame_id          = detection.name;
            output.transform.translation.x = tvec.at<double>(0, 0);
            output.transform.translation.y = tvec.at<double>(1, 0);
            output.transform.translation.z = tvec.at<double>(2, 0);
            output.transform.rotation.w    = quat.w();
            output.transform.rotation.x    = quat.x();
            output.transform.rotation.y    = quat.y();
            output.transform.rotation.z    = quat.z();

            outputs.transforms.push_back(output);
        }

        output_pub_->publish(outputs);
    }

    void info_callback(sensor_msgs::msg::CameraInfo::UniquePtr msg) {
        this->camera_info_ = *msg;
    }

  private:
    rclcpp::Subscription<guppy_msgs::msg::CornerDetectionList>::SharedPtr
                                                                  corner_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;

    rclcpp::Publisher<guppy_msgs::msg::TransformList>::SharedPtr output_pub_;

    sensor_msgs::msg::CameraInfo camera_info_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<PerspectiveNPoint>());

    rclcpp::shutdown();

    return 0;
}
