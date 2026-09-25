#ifndef CAN_HPP
#define CAN_HPP

#include "rclcpp/logger.hpp"

#include <linux/can.h>
#include <string_view>

class Socket {
private:
    const rclcpp::Logger logger_ = rclcpp::get_logger("Socket");
    unsigned int         descriptor_;
    bool                 active_;
public:
    Socket(std::string_view interface, rclcpp::Logger logger);
    Socket(const Socket&)            = delete;
    Socket& operator=(const Socket&) = delete;
    Socket(Socket&& other);
    Socket& operator=(Socket&& other);
    ~Socket();
    bool read(can_frame* frame, size_t* bytes_read = nullptr) const;
    bool write(const can_frame* frame, size_t* bytes_written = nullptr) const;
private:
    bool initialize_socket(std::string_view interface);
};

#endif
