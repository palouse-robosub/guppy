#include "guppy_msgs/srv/send_can.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstdio>
#include <cstring>
#include <memory>

#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

static int sock_ = -1;

bool setup_can_socket() {
  sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock_ < 0) {
    perror("socket");
    return false;
  }

  ifreq ifr{};
  const char* can_net = "can0";
  std::strncpy(ifr.ifr_name, can_net, IFNAMSIZ - 1);

  if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
    perror("SIOCGIFINDEX");
    return false;
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    perror("bind");
    return false;
  }

  return true;
}

void send(const std::shared_ptr<guppy_msgs::srv::SendCan::Request> request, std::shared_ptr<guppy_msgs::srv::SendCan::Response> response) {
  can_frame frame{};
  frame.can_id = request->id;
  frame.can_dlc = request->data.size();

  if (frame.can_dlc > 8) {
    RCLCPP_ERROR(rclcpp::get_logger("can_tx"), "CAN payload too large (%zu bytes)", request->data.size());
    response->written = -1;
    return;
  }

  std::memcpy(frame.data, request->data.data(), frame.can_dlc);

  ssize_t nbytes = write(sock_, &frame, sizeof(frame));
  response->written = nbytes;

  if (nbytes < 0) {
    perror("write");
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  if (!setup_can_socket()) {
    RCLCPP_FATAL(rclcpp::get_logger("can_tx"), "Failed to setup CAN socket");
    return 1;
  }

  auto node = rclcpp::Node::make_shared("can_tx");

  auto service = node->create_service<guppy_msgs::srv::SendCan>(
      "can_tx", [](const std::shared_ptr<guppy_msgs::srv::SendCan::Request> request, std::shared_ptr<guppy_msgs::srv::SendCan::Response> response) { send(request, response); }, 10);

  RCLCPP_INFO(node->get_logger(), "CAN TX service ready");

  rclcpp::spin(node);

  close(sock_);
  rclcpp::shutdown();
  return 0;
}
