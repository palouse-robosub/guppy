#include "guppy_testing/fake_socket.hpp"

#include <variant>

namespace guppy_testing {

void FakeSocket::inject(const frame& frame) {
    {
        std::lock_guard lock(m_mutex);
        m_rx.push_back(frame);
    }
    m_cv.notify_one();
}

std::vector<FakeSocket::frame> FakeSocket::sent() const {
    std::lock_guard lock(m_mutex);
    return m_tx;
}

std::optional<FakeSocket::frame> FakeSocket::receive() {
    std::unique_lock lock(m_mutex);
    m_cv.wait(
        lock,
        [this]() {
            return m_stopped || !m_rx.empty();
        }
    );
    if (m_stopped)
        throw std::runtime_error("socket stopped!");
    auto frame = m_rx.front();
    m_rx.pop_front();
    return frame;
}

std::size_t FakeSocket::send(const can_frame& frame) {
    return record(frame);
}

std::size_t FakeSocket::send(const canfd_frame& frame) {
    return record(frame);
};

void FakeSocket::stop() {
    {
        std::lock_guard lock(m_mutex);
        m_stopped = true;
    }
    m_cv.notify_all();
}

} // namespace guppy_testing
