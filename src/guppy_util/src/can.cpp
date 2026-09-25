#include "guppy_util/can.hpp"

#include "rclcpp/logging.hpp"

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

// public members
Socket::Socket(std::string_view interface, rclcpp::Logger logger)
: logger_(std::move(logger)) {
    active_ = initialize_socket(interface);
}

Socket::Socket(Socket&& other)
: logger_(other.logger_), descriptor_(other.descriptor_), active_(other.active_) {
    other.active_ = false;
}

Socket& Socket::operator=(Socket&& other) {
    if (this != &other) {
        if (active_)
            ::close(static_cast<int>(descriptor_));
        descriptor_ = other.descriptor_;
        active_ = other.active_;
        other.active_ = false;
    }
    return *this;
}

Socket::~Socket() {
    if (active_)
        ::close(static_cast<signed int>(descriptor_));
}

bool Socket::read(can_frame* frame, size_t* bytes_read) const {
    if (!active_) {
        RCLCPP_WARN(logger_, "Cannot read from inactive socket!");
        return false;
    }
    const auto size = ::read(descriptor_, frame, sizeof(can_frame));
    if (size < 0) {
        RCLCPP_WARN(logger_, "Reading from CAN socket failed: %s", std::strerror(errno));
        return false;
    }
    if (bytes_read)
        *bytes_read = static_cast<size_t>(size);
    return true;
}

bool Socket::write(const can_frame* frame, size_t* bytes_written) const {
    if (!active_) {
        RCLCPP_WARN(logger_, "Cannot write to inactive socket!");
        return false;
    }
    const auto size = ::write(descriptor_, frame, sizeof(*frame));
    if (size < 0) {
        RCLCPP_ERROR(logger_, "Failed to write to CAN socket!");
        return false;
    }
    if (static_cast<unsigned long>(size) != sizeof(*frame)) {
        RCLCPP_ERROR(logger_, "Write mismatch over CAN socket!");
        return false;
    }
    if (bytes_written)
        *bytes_written = static_cast<size_t>(size);
    return true;
}

// private methods
bool Socket::initialize_socket(std::string_view interface) {
    auto new_descriptor = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (new_descriptor < 0) {
        RCLCPP_FATAL(logger_, "Failed to create CAN socket: %s", std::strerror(errno));
        return false;
    }
    ifreq request{};
    std::strncpy(request.ifr_name, std::string(interface).c_str(), IFNAMSIZ);
    request.ifr_name[IFNAMSIZ - 1] = '\0';
    if(ioctl(new_descriptor, SIOCGIFINDEX, &request) < 0) {
        RCLCPP_FATAL(logger_, "Failed to get interface index for '%s': %s", request.ifr_name, std::strerror(errno));
        ::close(new_descriptor);
        return false;
    }
    sockaddr_can address{};
    address.can_family = AF_CAN, address.can_ifindex = request.ifr_ifindex;
    if (bind(new_descriptor, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) < 0) {
        RCLCPP_FATAL(logger_, "Failed to bind CAN socket to '%s': %s", request.ifr_name, std::strerror(errno));
        ::close(new_descriptor);
        return false;
    }
    descriptor_ = static_cast<unsigned int>(new_descriptor);
    return true;
}
