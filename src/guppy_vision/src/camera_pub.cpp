#include <CameraDefs.h>
#include <Spinnaker.h>
#include <atomic>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
// #include <SpinGenApi/SpinnakerGenApi.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/fill_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"

typedef struct camInfo {
  std::string serial;
  std::string position;
} CamInfo;

std::vector<CamInfo> cams = {
    {.serial = "17473425", .position = "fl"}, {.serial = "17473424", .position = "fr"}, {.serial = "14406636", .position = "d"},  {.serial = "14406334", .position = "rl"},
    {.serial = "14406637", .position = "rr"}, {.serial = "20188596", .position = "cl"}, {.serial = "20188587", .position = "cr"},
};

class FlirPublisher : public rclcpp::Node {
 public:
  FlirPublisher() : Node("camera_publisher") {
    this->running_.store(true);

    this->system = Spinnaker::System::GetInstance();
    this->camList = system->GetCameras();

    std::string base = "/cam/";

    for (unsigned int i = 0; i < camList.GetSize(); ++i) {
        std::string pos = "unknown";
      for (CamInfo &c: cams) {
          if (this->camList[i]->GetDeviceSerialNumber().c_str() == c.serial) {
              pos = c.position;
              break;
          }
      }
      std::string topic = "/cam/" + pos;
      this->pubs_.push_back(image_transport::create_publisher(this, topic));
      // this->publishers_.push_back(this->create_publisher<sensor_msgs::msg::Image>(topic, 1));
      this->threads_.emplace_back(&FlirPublisher::acquireFootage, this, i);
    }
  }

  ~FlirPublisher() {
    this->running_.store(false);

    for (auto& t : threads_) {
      if (t.joinable()) {
        t.join();
      }
    }

    this->camList.Clear();
    this->system->ReleaseInstance();
  }

 private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  std::vector<image_transport::Publisher> pubs_;

  Spinnaker::SystemPtr system;
  Spinnaker::CameraList camList;

  std::thread ackThread;
  std::atomic<bool> running_{false};

  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> publishers_;
  std::vector<std::thread> threads_;

  void acquireFootage(int camIndex) {
    Spinnaker::CameraPtr pCam = this->camList[camIndex];
    pCam->Init();
    pCam->BeginAcquisition();
    Spinnaker::ImageProcessor processor;
    while (this->running_) {
      Spinnaker::ImagePtr pImg = pCam->GetNextImage(1000);
      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      sensor_msgs::fillImage(*msg, sensor_msgs::image_encodings::BAYER_RGGB8, pImg->GetHeight(), pImg->GetWidth(), pImg->GetStride(), pImg->GetData());
      pImg->Release();

      this->pubs_[camIndex].publish(*msg.get());
      // this->publishers_[camIndex]->publish(std::move(msg));
    }
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlirPublisher>());
  rclcpp::shutdown();
  return 0;
}
