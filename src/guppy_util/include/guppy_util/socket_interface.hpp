#ifndef GUPPY_UTIL_SOCKET_INTERFACE_HPP
#define GUPPY_UTIL_SOCKET_INTERFACE_HPP

#include <linux/can.h>

#include <optional>
#include <variant>

namespace guppy_util {

class ISocket {
public:
    using frame = std::variant<can_frame, canfd_frame>;

    virtual ~ISocket() = default;
    [[nodiscard]] virtual std::optional<frame> receive() = 0;
    virtual size_t send(const can_frame& frame) = 0;
    virtual size_t send(const canfd_frame& frame) = 0;
    virtual void stop() = 0;
};

} // namespace gupy_util

#endif // GUPPY_UTIL_SOCKET_INTERFACE_HPP
