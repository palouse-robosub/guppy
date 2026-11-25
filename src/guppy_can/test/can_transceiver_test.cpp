#include <cstdint>
#include <linux/can.h>
#include <gtest/gtest.h>

#include <algorithm>

#include <rclcpp/node_options.hpp>
#include <rclcpp/executors.hpp>

#include "guppy_can/can_transceiver.hpp"
#include "guppy_util/socket_interface.hpp"
#include "guppy_util/can.hpp"
#include "guppy_util/socket.hpp"
#include "guppy_testing/can_peer.hpp"
#include "guppy_util/quality.hpp"

using namespace std::chrono_literals;

constexpr auto interface = "vcan0";

#ifdef ENABLE_CAN_FD
constexpr size_t max_data_length = CANFD_MAX_DLEN;
#else
constexpr size_t max_data_length = CAN_MAX_DLEN;
#endif

constexpr auto test_name_one = "CurrentState";
constexpr auto test_name_two = "MotorBoardHeartbeat";

namespace guppy_can {

class TestCANTransceiver : public CANTransceiver {
public:
    using CANTransceiver::CANTransceiver;

    const Registry& registry() const {
        return m_registry;
    }
};

template <typename Frame>
Frame make_frame(canid_t id, const std::vector<uint8_t>& bytes) {
    Frame frame{};
    frame.can_id = id;
    frame.len = static_cast<uint8_t>(bytes.size());
    std::copy(bytes.begin(), bytes.end(), frame.data);
    return frame;
}

canfd_frame as_fd(const guppy_util::ISocket::frame& variant) {
    return std::visit(
        [](const auto& frame) {
            canfd_frame out{};
            out.can_id = frame.can_id;
            out.len = frame.len;
            std::copy(frame.data, frame.data + frame.len, out.data);
            return out;
        },
        variant
    );
}

class CANTransceiverFixture : public testing::Test {
protected:
    using send_can = guppy_msgs::srv::SendCAN;
    using msg = guppy_msgs::msg::CANFrame;

    std::unique_ptr<guppy_testing::CANPeer> m_peer;
    std::shared_ptr<TestCANTransceiver> m_node;
    std::shared_ptr<rclcpp::Node> m_client_node;
    rclcpp::executors::SingleThreadedExecutor m_executor;

    // testing constants
    canid_t resolved_id_one;
    canid_t resolved_id_two;

    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }

    void SetUp() override {
        m_peer = std::make_unique<guppy_testing::CANPeer>(interface);
        m_node = std::make_shared<TestCANTransceiver>(std::make_unique<guppy_util::Socket>(interface), rclcpp::NodeOptions{});
        m_client_node = std::make_shared<rclcpp::Node>("can_test_client");
        m_executor.add_node(m_node);
        m_executor.add_node(m_client_node);

        resolved_id_one = m_node->registry().find(test_name_one)->id.id;
        resolved_id_two = m_node->registry().find(test_name_one)->id.id;
    }

    void TearDown() override {
        m_executor.remove_node(m_client_node);
        m_executor.remove_node(m_node);
        m_node.reset();
    }

    template <typename Predicate>
    bool spin_until(Predicate predicate, std::chrono::milliseconds timeout = 3s) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        // check predicate, if not true spin for 10ms and check again
        // return early if predicate true or repeat until deadline
        while (std::chrono::steady_clock::now() < deadline) {
            if (predicate())
                return true;
            m_executor.spin_some(10ms);
        }
        return predicate();
    }

    std::shared_ptr<send_can::Response> call_send(const std::string& name, std::vector<uint8_t> data) {
        auto client = m_client_node->create_client<send_can>("send_can", guppy_util::quality::volatile_profile);
        EXPECT_TRUE(client->wait_for_service(3s));
        auto request = std::make_shared<send_can::Request>();
        request->name = name;
        request->data = std::move(data);
        auto future = client->async_send_request(request);
        // blocking call that will spin until future completes, or timeout hits
        if (
            !spin_until(
                [&future]() {
                    return future.wait_for(0s) == std::future_status::ready;
                }
            )
        )
            return nullptr;
        return future.get();
    }

    std::shared_ptr<rclcpp::Subscription<msg>> subscribe(canid_t id, std::optional<msg>& out) {
        return m_client_node->create_subscription<msg>(
            guppy_util::format_can_topic(id), guppy_util::quality::volatile_profile,
            [&out](std::shared_ptr<const msg> message) {
                out = *message;
            }
        );
    }

    template <typename Frame>
    bool deliver(const Frame& frame, const std::optional<msg>& out) {
        return spin_until(
            [&]() {
                if (!out)
                    m_peer->send(frame);
                return out.has_value();
            }
        );
    }

    void expect_on_bus(canid_t id, const std::vector<uint8_t>& data) {
        const auto received = m_peer->read(1s);
        ASSERT_TRUE(received.has_value()) << "nothing arrived on bus";
        const auto frame = as_fd(*received);
        EXPECT_EQ(frame.can_id, id);
        EXPECT_EQ(frame.len, data.size());
        EXPECT_TRUE(std::equal(data.begin(), data.end(), frame.data));
    }
};


TEST_F(CANTransceiverFixture, SendFrameToBus) {
    const std::vector<uint8_t> data{0x0B, 0x0E, 0x0E, 0x0F};
    const auto response = call_send(test_name_one, data);
    ASSERT_TRUE(response);
    EXPECT_TRUE(response->success);
    expect_on_bus(resolved_id_one, data);
}


TEST_F(CANTransceiverFixture, SendAcceptsMaxPayload) {
    const std::vector<uint8_t> data(max_data_length, 0xFF);
    const auto response = call_send(test_name_one, data);
    ASSERT_TRUE(response);
    EXPECT_FALSE(response->success);
    expect_on_bus(resolved_id_one, data);
}

TEST_F(CANTransceiverFixture, SendServiceReportsSocketFailure) {
    const auto response = call_send(test_name_one, std::vector<uint8_t>(max_data_length + 1, 0xFF));
    ASSERT_TRUE(response);
    EXPECT_FALSE(response->success);
    EXPECT_FALSE(m_peer->read(100ms).has_value()) << "frame reached bus anyway";
}

TEST_F(CANTransceiverFixture, PublishesClassicalFrame) {
    std::optional<msg> received;
    auto subscriber = subscribe(resolved_id_one, received);
    const std::vector<uint8_t> data{0xAB, 0xCD};

    ASSERT_TRUE(deliver(make_frame<can_frame>(resolved_id_one, data), received));
    EXPECT_EQ(received->can_id, resolved_id_one);
    EXPECT_EQ(received->len, 2);
    EXPECT_EQ(received->data, data);
}

#ifdef ENABLE_CAN_FD
TEST_F(CANTransceiverFixture, PublishesFlexibleFrame) {
    std::optional<msg> received;
    auto subscriber = subscribe(resolved_id_one, received);
    const std::vector<uint8_t> data(32, 0xFF);

    ASSERT_TRUE(deliver(make_frame<canfd_frame>(resolved_id_one, data), received));
    EXPECT_EQ(received->can_id, resolved_id_one);
    EXPECT_EQ(received->len, 2);
    EXPECT_EQ(received->data, data);
}
#endif

TEST_F(CANTransceiverFixture, MultipleTopicsForDifferentID) {
    std::optional<msg> received_one, received_two;
    auto subscriber_one = subscribe(resolved_id_one, received_one);
    auto subscriber_two = subscribe(resolved_id_two, received_two);

    ASSERT_TRUE(deliver(make_frame<can_frame>(resolved_id_one, {0xAA, 0xBB}), received_one));
    EXPECT_FALSE(received_two.has_value());

    ASSERT_TRUE(deliver(make_frame<can_frame>(resolved_id_two, {0xCC, 0xDD}), received_two));

    EXPECT_EQ(received_one->can_id, resolved_id_one);
    EXPECT_EQ(received_one->data, (std::vector<uint8_t>{0xAA, 0xBB}));
    EXPECT_EQ(received_two->can_id, resolved_id_two);
    EXPECT_EQ(received_two->data, (std::vector<uint8_t>{0xCC, 0xDD}));
}

} // namespace guppy_can
