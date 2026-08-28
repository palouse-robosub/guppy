#include <atomic>
#include <cstdio>
#include <string>
#include <thread>

#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "guppy_msgs/msg/can_frame.hpp"
#include "rclcpp/rclcpp.hpp"

#define MAX_PUBLISHERS 100

constexpr uint32_t CAN_ID_BIT_MASK = 1u << 10;

using namespace std::chrono_literals;

class CanRx : public rclcpp::Node {
 public:
  CanRx() : Node("can_rx") {
    this->declare_parameter("interface", "can0");  // name of CAN interface
    if (!setup_can_socket()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to setup CAN socket");
      return;
    }
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    auto cb = [this](const rclcpp::Parameter& p) {
      if (!setup_can_socket()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to setup CAN socket");
        return;
      }

      RCLCPP_INFO(this->get_logger(), "cb: Received an update to parameter \"%s\" of type %s: \"%s\"", p.get_name().c_str(), p.get_type_name().c_str(), p.as_string().c_str());
    };
    cb_handle_ = param_subscriber_->add_parameter_callback("interface", cb);
    running_.store(true);
    can_thread_ = std::thread(&CanRx::can_loop, this);
  }

  ~CanRx() override {
    running_.store(false);
    if (sock_ >= 0)
      close(sock_);
    if (can_thread_.joinable())
      can_thread_.join();
  }

 private:
  int sock_ = -1;
  int ids_[MAX_PUBLISHERS] = {0};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<guppy_msgs::msg::CanFrame>::SharedPtr publishers_[MAX_PUBLISHERS];
  std::thread can_thread_;
  std::atomic<bool> running_{false};

  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;

  bool setup_can_socket() {
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Socket creation failed.");
      return false;
    }

    ifreq ifr{};
    const char* can_net = this->get_parameter("interface").as_string().c_str();
    std::strcpy(ifr.ifr_name, can_net);
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(this->get_logger(), "SIOCGIFINDEX failed.");
      return false;
    }

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "CAN bind failed.");
      return false;
    }

    return true;
  }

  // checks if a publisher has been created
  int check_id(const int id) const {
    for (int i = 0; i < MAX_PUBLISHERS; i++) {
      if (ids_[i] == id)
        return i;
    }
    return -1;
  }

  // publishes data from can frame to ros topic
  void publish_bytes(const int idx, __u8 data[], const int len) const {
    guppy_msgs::msg::CanFrame frame;
    frame.can_id = ids_[idx];
    frame.len = len;
    frame.data.assign(data, data + len);
    frame.stamp = this->now();
    publishers_[idx]->publish(frame);

    RCLCPP_DEBUG(this->get_logger(), "0x%03X [%d] ", ids_[idx], len);
    for (int i = 0; i < len; i++)
      RCLCPP_DEBUG(this->get_logger(), "%02X ", data[i]);
    RCLCPP_DEBUG(this->get_logger(), "\r\n");
  }

  // creates a ros publisher for a given can id
  int create_id_publisher(const int id) {
    for (int i = 0; i < MAX_PUBLISHERS; i++) {
      if (ids_[i] == 0) {
        ids_[i] = id;
        const std::string id_str = int_to_hex_str(id);
        const std::string base = "/can/id_0x";
        const std::string topic_str = base + id_str;
        publishers_[i] = this->create_publisher<guppy_msgs::msg::CanFrame>(topic_str, 10);
        RCLCPP_INFO(this->get_logger(), "created publisher %s\r\n", topic_str.c_str());
        return i;
      }
    }
    return -1;
  }

  static std::string int_to_hex_str(int value) {
    char buffer[20];
    sprintf(buffer, "%x", value);
    return {buffer};
  }

  void can_loop() {
    while (running_.load()) {
      can_frame frame{};

      const int nbytes = read(sock_, &frame, sizeof(can_frame));

      // checks for read error
      if (nbytes < 0) {
        RCLCPP_WARN(this->get_logger(), "Reading from CAN frame failed");
        continue;
      }

      // do not publish to ros if msb is 1
      if (frame.can_id & CAN_ID_BIT_MASK) {
        continue;
      }

      if (int idx = check_id(frame.can_id); idx != -1) {
        publish_bytes(idx, frame.data, frame.can_dlc);
      } else {
        idx = create_id_publisher(frame.can_id);
        if (idx != -1) {
          publish_bytes(idx, frame.data, frame.can_dlc);
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to create publisher for CAN ID %x", frame.can_id);
        }
      }
    }
  }
};

int main(const int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<CanRx>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
