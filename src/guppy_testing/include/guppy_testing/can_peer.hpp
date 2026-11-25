#ifndef GUPPY_TESTING_CAN_PEER_HPP
#define GUPPY_TESTING_CAN_PEER_HPP

#include <chrono>
#include <optional>

#include "guppy_util/socket.hpp"

namespace guppy_testing {

class CANPeer : public guppy_util::Socket {
public:
    using Socket::Socket;

    std::optional<frame> read(std::chrono::milliseconds timeout);
};

} // namespace guppy_testing

#endif
