#ifndef GUPPY_TESTING_FAKE_SOCKET_HPP
#define GUPPY_TESTING_FAKE_SOCKET_HPP

#include <condition_variable>
#include <deque>
#include <mutex>
#include <vector>
#include <optional>
#include <stdexcept>

#include "guppy_util/socket_interface.hpp"

namespace guppy_testing {

class FakeSocket : public guppy_util::ISocket {
private:
    mutable std::mutex m_mutex;
    std::condition_variable m_cv;
    std::deque<frame> m_rx;
    std::vector<frame> m_tx;
    bool m_stopped = false;
public:
    bool fail_sends = false;

    void inject(const frame& frame);
    std::vector<frame> sent() const;

    std::optional<frame> receive() override;
    std::size_t send(const can_frame& frame) override;
    std::size_t send(const canfd_frame& frame) override;

    void stop() override;

    template <typename Frame>
    size_t record(const Frame& frame) {
        if (fail_sends)
            throw std::runtime_error("send failed!");
        std::lock_guard lock(m_mutex);
        m_tx.push_back(frame);
        return sizeof(frame);
    }
};

} // namespace guppy_testing

#endif // GUPPY_TESTING_FAKE_SOCKET_HPP
