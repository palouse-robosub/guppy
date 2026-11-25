#include "guppy_util/socket.hpp"

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can/raw.h>
#include <bits/sockaddr.h>
#include <fcntl.h>
#include <net/if.h>
#include <unistd.h>
#include <sys/eventfd.h>
#include <sys/poll.h>

#include <cerrno>
#include <stdexcept>
#include <system_error>
#include <cstring>

namespace guppy_util {

// nested class
Socket::FileDescriptor::FileDescriptor(int file_descriptor) : m_file_descriptor(file_descriptor) { }

Socket::FileDescriptor::~FileDescriptor() {
    // https://www.man7.org/linux/man-pages/man2/close.2.html
    if (m_file_descriptor != -1)
        close(m_file_descriptor);
}

int Socket::FileDescriptor::get() const noexcept {
    return m_file_descriptor;
}

// public members
Socket::Socket(std::string_view interface) : m_shutdown(eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK)), m_socket(socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW)) {
    // https://www.man7.org/linux/man-pages/man2/socket.2.html
    if (m_socket.get() < 0)
        throw std::system_error(errno, std::generic_category(), "failed to create can socket!");
    // https://www.man7.org/linux/man-pages/man2/eventfd.2.html
    if (m_shutdown.get() < 0)
        throw std::system_error(errno, std::generic_category(), "failed to create shutdown file descriptor!");

    // https://www.man7.org/linux/man-pages/man7/netdevice.7.html
    struct ifreq interface_request{};
    if (interface.empty() || interface.size() >= IFNAMSIZ)
        throw std::invalid_argument("invalid interface name!");
    std::copy(interface.begin(), interface.end(), std::begin(interface_request.ifr_name));

    // https://www.man7.org/linux/man-pages/man2/ioctl.2.html
    if (ioctl(m_socket.get(), SIOCGIFINDEX, &interface_request) == -1)
        throw std::system_error(errno, std::generic_category(), "failed to get interface index!");

    struct sockaddr_can socket_address{};
    socket_address.can_family = static_cast<sa_family_t>(AF_CAN);
    socket_address.can_ifindex = interface_request.ifr_ifindex;

    // https://www.man7.org/linux/man-pages/man2/bind.2.html
    if (bind(m_socket.get(), reinterpret_cast<struct sockaddr *>(&socket_address), sizeof(socket_address)) == -1)
        throw std::system_error(errno, std::generic_category(), "failed to bind address to can socket!");

    constexpr int enable_canfd = 1;
    if (setsockopt(m_socket.get(), SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) == -1)
        throw std::system_error(errno, std::generic_category(), "failed to enable can fd frames!");
}

size_t Socket::send(const can_frame& frame) {
    const auto size = write(m_socket.get(), &frame, CAN_MTU);
    if (size == -1)
        throw std::system_error(errno, std::generic_category(), "failed to send classical can frame!");
    if (size != CAN_MTU)
        throw std::runtime_error("incomplete classical can frame write!");
    return static_cast<size_t>(size);
}

size_t Socket::send(const canfd_frame& frame) {
    const auto size = write(m_socket.get(), &frame, CANFD_MTU);
    if (size == -1)
        throw std::system_error(errno, std::generic_category(), "failed to send can fd frame!");
    if (size != CANFD_MTU)
        throw std::runtime_error("incomplete can fd frame write!");
    return static_cast<size_t>(size);
}

std::optional<Socket::frame> Socket::receive() {
    while (true) {
        if (!wait())
            return std::nullopt;
        struct canfd_frame flexible_frame{};
        const auto size = read(m_socket.get(), &flexible_frame, CANFD_MTU);
        // https://www.man7.org/linux/man-pages/man2/read.2.html
        if (size == -1) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
                continue;
            throw std::system_error(errno, std::generic_category(), "failed to read from can socket!");
        }
        if (size == CAN_MTU) {
            can_frame classical_frame{};
            std::memcpy(&classical_frame, &flexible_frame, CAN_MTU);
            return classical_frame;
        }
        if (size == CANFD_MTU)
            return flexible_frame;
        throw std::runtime_error("unexpected can frame size!");
    }
}

void Socket::stop() {
    const uint64_t value = 1;
    const auto size = write(m_shutdown.get(), &value, sizeof(value));
    if (size == -1 && errno != EAGAIN)
        throw std::system_error(errno, std::generic_category(), "failed to signal socket shutdown!");
}

// private members
bool Socket::wait() const {
    struct pollfd poll_requests[2]{};
    poll_requests[0].fd = m_socket.get();
    poll_requests[0].events = POLLIN;
    poll_requests[1].fd = m_shutdown.get();
    poll_requests[1].events = POLLIN;

    while (true) {
        // https://www.man7.org/linux/man-pages/man2/poll.2.html
        const int result = ppoll(poll_requests, 2, nullptr, nullptr);
        if (result == -1) {
            if (errno != EINTR)
                throw std::system_error(errno, std::generic_category(), "failed to wait for can frame!");
            continue;
        }
        if (poll_requests[1].revents & POLLIN)
            return false; // shutdown
        if (poll_requests[0].revents & POLLIN)
            return true; // can frame in buffer
        if ((poll_requests[0].revents | poll_requests[1].revents) & (POLLERR | POLLHUP | POLLNVAL))
            throw std::runtime_error("can socket poll error!");
    }
}

} // namespace guppy_util
