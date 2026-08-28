#include "guppy_control/t200_interface.hpp"

namespace t200_interface {

bool T200Interface::setup_can() {
  sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock_ < 0)
    return false;

  struct ifreq ifr;
  std::strcpy(ifr.ifr_name, can_interface_.c_str());
  if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0)
    return false;

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(sock_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
    return false;

  return true;
}

bool T200Interface::send_to_can(unsigned int can_id, float value) {
  struct can_frame frame;
  frame.can_id = can_id;
  frame.can_dlc = sizeof(float);

  std::memcpy(frame.data, &value, sizeof(float));

  if (::write(sock_, &frame, sizeof(struct can_frame)) < 0)
    return false;
  return true;
}

bool T200Interface::write(Eigen::VectorXd throttles) {
  size_t n = throttles.size();
  if (can_ids.size() < n)
    return false;

  bool okay = true;
  for (size_t i = 0; i < n; i++) {
    if (this->enabled_)
      okay = okay && send_to_can(can_ids[i], throttles[i]);
    else
      okay = okay && send_to_can(can_ids[i], 0);
    // std::cout << can_ids[i] << "\t" << throttles[i] << "\t" << okay <<
    // std::endl;
  }

  return okay;
}

bool T200Interface::initialize() {
  bool okay = true;
  for (int i = 0; i < 100; i++) {
    for (auto id : can_ids) {
      okay = okay && send_to_can(id, 0);
    }
  }
  return okay;
}

bool T200Interface::shutdown() {
  bool okay = true;
  for (int i = 0; i < 100; i++) {
    for (auto id : can_ids) {
      okay = okay && send_to_can(id, 0);
    }
  }
  return okay;
}

void T200Interface::set_enabled(bool enabled) {
  this->enabled_ = enabled;
}

}  // namespace t200_interface