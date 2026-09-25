#ifndef T200_INTERFACE_H
#define T200_INTERFACE_H

#include "guppy_util/can.hpp"

#include <array>
#include <rclcpp/logger.hpp>

class T200Interface {
public:
    static constexpr const inline unsigned int motor_count = 8;
private:
    static constexpr const inline unsigned int send_attempts = 100;
private:
    const std::array<canid_t, T200Interface::motor_count> can_ids_;
    const Socket                                          socket_;
    const rclcpp::Logger                                  logger_;
    bool                                                  enabled_;
public:
    T200Interface(
        std::string_view can_interface, std::array<canid_t, T200Interface::motor_count> can_ids,
        rclcpp::Logger logger
    );
    ~T200Interface();
    bool write(std::array<float, T200Interface::motor_count> throttles);
    void set_enabled(bool enabled);
private:
    bool send(canid_t can_id, float value);
    bool initialize();
    bool shutdown();
};

#endif
