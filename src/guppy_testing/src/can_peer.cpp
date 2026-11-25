#include "guppy_testing/can_peer.hpp"

#include <poll.h>

namespace guppy_testing {

std::optional<CANPeer::frame> CANPeer::read(std::chrono::milliseconds timeout) {
    pollfd descriptor{m_socket.get(), POLLIN, 0};
    if (poll(&descriptor, 1, static_cast<int>(timeout.count())) <= 0)
        return std::nullopt;
    return receive();
}

} // namespace guppy_testing
