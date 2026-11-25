#include <linux/can.h>
#include <gtest/gtest.h>

#include <algorithm>

#include <rclcpp/node_options.hpp>
#include <rclcpp/executors.hpp>

#include "guppy_can/can_transceiver.hpp"
#include "guppy_util/socket_interface.hpp"
#include "guppy_util/can.hpp"
#include "guppy_testing/fake_socket.hpp"

using namespace std::chrono_literals;

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

class CANTransceiverFixture : public testing::Test {
protected:
    using send_can = guppy_msgs::srv::SendCAN;
    using msg = guppy_msgs::msg::CANFrame;

    guppy_testing::FakeSocket* m_socket;
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
        m_socket = new guppy_testing::FakeSocket();
        m_node = std::make_shared<TestCANTransceiver>(std::unique_ptr<guppy_util::ISocket>(m_socket), rclcpp::NodeOptions{});
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
};

TEST_F(CANTransceiverFixture, SendServiceWritesFrameToSocket) {
    auto response = call_send(test_name_one, {0x0B, 0x0E, 0x0E, 0x0F});
    ASSERT_TRUE(response);
    EXPECT_TRUE(response->success);

    const auto sent = m_socket->sent();
    ASSERT_EQ(sent.size(), 1U);
    const auto& frame = std::get<canfd_frame>(sent.front());
    EXPECT_EQ(frame.can_id, 0x1337U);
    ASSERT_EQ(frame.len, 4);
    EXPECT_EQ(frame.data[2], 0xE);
}

TEST_F(CANTransceiverFixture, SendServiceRejectsOversizedPayload) {
    auto response = call_send(test_name_one, std::vector<uint8_t>(max_data_length + 1, 0xFF));
    ASSERT_TRUE(response);
    EXPECT_FALSE(response->success);
    EXPECT_TRUE(m_socket->sent().empty());
}

TEST_F(CANTransceiverFixture, SendServiceReportsSocketFailure) {
    m_socket->fail_sends = true;
    auto response = call_send(test_name_one, {0x01});
    ASSERT_TRUE(response);
    EXPECT_FALSE(response->success);
}

TEST_F(CANTransceiverFixture, TransceiverPublishesFrameFromSocket) {
    std::optional<msg> received;
    const auto topic = guppy_util::format_can_topic(test_name_one);
    auto subscriber = m_client_node->create_subscription<msg>(
        topic, guppy_util::quality::volatile_profile,
        [&received](std::shared_ptr<const msg> msg) {
            received = *msg;
        }
    );

    const std::vector<uint8_t> data(max_data_length, 0xFF);

    can_frame frame{};
    frame.can_id = resolved_id_one;
    frame.len = max_data_length;
    std::copy(data.begin(), data.end(), frame.data);

    ASSERT_TRUE(
        spin_until(
            [this, &received, &frame]() {
                if (!received)
                    m_socket->inject(frame);
                return received.has_value();
            }
        )
    );
    EXPECT_EQ(received->can_id, resolved_id_one);
    EXPECT_EQ(received->len, max_data_length);
    EXPECT_EQ(received->data, data);
}

TEST_F(CANTransceiverFixture, MultipleTopicsForDifferentID) {
    std::optional<msg> received_one;
    std::optional<msg> received_two;

    const auto topic_one = guppy_util::format_can_topic(test_name_one);
    const auto topic_two = guppy_util::format_can_topic(test_name_two);

    auto subscriber_one = m_client_node->create_subscription<msg>(
        topic_one,
        guppy_util::quality::volatile_profile,
        [&received_one](std::shared_ptr<const msg> frame) {
            received_one = *frame;
        }
    );

    auto subscriber_two = m_client_node->create_subscription<msg>(
        topic_two,
        guppy_util::quality::volatile_profile,
        [&received_two](std::shared_ptr<const msg> frame) {
            received_two = *frame;
        }
    );

    can_frame frame_one{};
    frame_one.can_id = resolved_id_one;
    frame_one.len = 2;
    frame_one.data[0] = 0xAA;
    frame_one.data[1] = 0xBB;

    can_frame frame_two{};
    frame_two.can_id = resolved_id_two;
    frame_two.len = 2;
    frame_two.data[0] = 0xCC;
    frame_two.data[1] = 0xDD;

    ASSERT_TRUE(
        spin_until(
            [this, &received_one, &frame_one]() {
                if (!received_one)
                    m_socket->inject(frame_one);
                return received_one.has_value();
            }
        )
    );

    EXPECT_FALSE(received_two.has_value());

    ASSERT_TRUE(
        spin_until(
            [this, &received_two, &frame_two]() {
                if (!received_two)
                    m_socket->inject(frame_two);
                return received_two.has_value();
            }
        )
    );

    ASSERT_TRUE(received_one.has_value());
    ASSERT_TRUE(received_two.has_value());

    EXPECT_EQ(received_one->can_id, resolved_id_one);
    EXPECT_EQ(received_one->len, 2);
    EXPECT_EQ(received_one->data, (std::vector<uint8_t>{0xAA, 0xBB}));

    EXPECT_EQ(received_two->can_id, resolved_id_two);
    EXPECT_EQ(received_two->len, 2);
    EXPECT_EQ(received_one->data, (std::vector<uint8_t>{0xCC, 0xDD}));
}

} // namespace guppy_can
