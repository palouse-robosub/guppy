#ifndef GUPPY_UTIL_SOCKET_HPP
#define GUPPY_UTIL_SOCKET_HPP

#include "guppy_util/socket_interface.hpp"

#include <linux/can.h>

#include <optional>
#include <string_view>

namespace guppy_util {

// https://www.kernel.org/doc/html/latest/networking/can.html
class Socket : public ISocket {
private:
    class FileDescriptor {
    private:
        int m_file_descriptor = -1;
    public:
        explicit FileDescriptor(int file_descriptor);
        ~FileDescriptor();

        FileDescriptor(const FileDescriptor&) = delete;
        FileDescriptor& operator=(const FileDescriptor&) = delete;

        [[nodiscard]] int get() const noexcept;
    };

    FileDescriptor m_shutdown;

    bool wait() const;
protected:
    FileDescriptor m_socket;
public:
    explicit Socket(std::string_view interface);

    Socket(const Socket&) = delete;
    Socket& operator=(const Socket&) = delete;

    size_t send(const can_frame& frame) override;
    size_t send(const canfd_frame& frame) override;

    [[nodiscard]] std::optional<frame> receive() override;

    void stop() override;
};

} // namespace guppy_util

#endif // GUPPY_UTIL_SOCKET_HPP
