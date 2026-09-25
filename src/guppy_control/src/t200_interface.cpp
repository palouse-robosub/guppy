#include "guppy_control/t200_interface.hpp"

#include <cstring>

// public methods
T200Interface::T200Interface(
    std::string_view can_interface, std::array<canid_t, T200Interface::motor_count> can_ids, rclcpp::Logger logger
) : can_ids_(can_ids), socket_(can_interface, logger), logger_(logger) {
    initialize();
}

T200Interface::~T200Interface() {
    shutdown();
}

bool T200Interface::write(std::array<float, T200Interface::motor_count> throttles) {
    auto okay = true;
    for (size_t i = 0; i < T200Interface::motor_count; i++) {
        if (!send(can_ids_[i], this->enabled_ ? throttles[i] : 0.0))
            okay = false;
    }
    return okay;
}

void T200Interface::set_enabled(bool enabled) {
    this->enabled_ = enabled;
}
// private methods
bool T200Interface::send(canid_t can_id, float value) {
    struct can_frame frame;
    frame.can_id  = can_id;
    frame.len = sizeof(float);
    std::memcpy(frame.data, &value, sizeof(float));
    return socket_.write(&frame);
}

bool T200Interface::initialize() {
    auto okay = true;
    for (unsigned int i = 0; i < T200Interface::send_attempts; i++)
        for (auto id : can_ids_) {
            if (!send(id, 0.0))
                okay = false;
        }
    return okay;
}

bool T200Interface::shutdown() {
    auto okay = true;
    for (unsigned int i = 0; i < T200Interface::send_attempts; i++)
        for (auto id : can_ids_) {
            if (!send(id, 0.0))
                okay = false;
        }
    return okay;
}
