#ifndef GUPPY_CAN_CAN_TRANSCEIVER_HPP
#define GUPPY_CAN_CAN_TRANSCEIVER_HPP

#include <memory>
#include <thread>
#include <atomic>
#include <vector>
#include <unordered_map>
#include <filesystem>
#include <string_view>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/logging.hpp>

#include "guppy_util/quality.hpp"
#include "guppy_util/socket_interface.hpp"
#include "guppy_msgs/msg/can_frame.hpp"
#include "guppy_msgs/srv/send_can.hpp"
#include "can_xml/common.hpp"

#define ENABLE_CAN_FD

namespace guppy_can {

class CANTransceiver : public rclcpp::Node {
protected:
class Registry {
public:
    static Registry load(const std::filesystem::path& file);
    Registry(const std::vector<can_xml::CANIDMeta> entries);
    Registry() = default;
    const can_xml::CANIDMeta* find(canid_t id) const;
    const can_xml::CANIDMeta* find(const std::string& name) const;
private:
    std::vector<can_xml::CANIDMeta> m_entries;
    std::unordered_map<canid_t, size_t> m_by_id;
    std::unordered_map<std::string, size_t> m_by_name;
};

    using publisher_shared_ptr = std::shared_ptr<rclcpp::Publisher<guppy_msgs::msg::CANFrame>>;
    using send_can = guppy_msgs::srv::SendCAN;

    std::unique_ptr<guppy_util::ISocket> m_socket;
    std::thread m_receiver_thread;
    std::atomic<bool> m_stop_thread{false};

    std::unordered_map<canid_t, publisher_shared_ptr> m_publishers;
    Registry m_registry{};

    const std::shared_ptr<const rclcpp::Service<send_can>> m_send_service = create_service<send_can>(
        "send_can",
        [this](
            const std::shared_ptr<send_can::Request>&  request,
            const std::shared_ptr<send_can::Response>& response
        ) {
            this->send(request, response);
        },
        guppy_util::quality::volatile_profile
    );
public:
    explicit CANTransceiver(std::unique_ptr<guppy_util::ISocket> socket, const rclcpp::NodeOptions& options);
    CANTransceiver(const rclcpp::NodeOptions& options);
    ~CANTransceiver();

    template<typename Socket>
    static std::shared_ptr<CANTransceiver> create(const rclcpp::NodeOptions& options) {
        auto node = std::make_shared<CANTransceiver>(options);
        std::string interface;
        node->declare_parameter<std::string>("interface", interface);
        node->m_socket = std::make_unique<Socket>(interface);
        node->start_receiver_thread();
        return node;
    }
protected:
    virtual void create_socket(std::string_view interface);
    void load_registry();

    void start_receiver_thread();
    void stop_receiver_thread();

    // creates a ros publisher for a given can id
    publisher_shared_ptr create_id_publisher(const can_xml::CANIDMeta* meta, canid_t id);

    // receiving loop in another thread
    void receive();
    void send(
        const std::shared_ptr<const send_can::Request>& request,
        const std::shared_ptr<send_can::Response>&      response
    );

    // publishes data from can frame to ros topic
    template <typename Frame>
    void publish_frame(publisher_shared_ptr& publisher, const Frame& frame, const can_xml::CANIDMeta*) const {
        // TODO use can_xml to format payload based on data type for nice printing
        guppy_msgs::msg::CANFrame msg;
        msg.can_id = frame.can_id;
        msg.len    = frame.len;
        msg.data.assign(frame.data, frame.data + frame.len);
        msg.stamp = this->now();
        publisher->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "0x%03X [%d] ", frame.can_id, static_cast<int>(frame.len));
        for (size_t i = 0; i < frame.len; i++)
            RCLCPP_DEBUG(this->get_logger(), "%02X ", frame.data[i]);
        RCLCPP_DEBUG(this->get_logger(), "\r\n");
    }

    // process received frame
    template <typename Frame>
    void process(const Frame& frame) {
        const auto* meta = m_registry.find(frame.can_id);
        if (meta && !meta->ros_publish)
            return;
        auto it = m_publishers.find(frame.can_id);
        if (it == m_publishers.end()) {
            if (auto publisher = create_id_publisher(meta, frame.can_id))
                publish_frame(publisher, frame, meta);
            else
                RCLCPP_ERROR(this->get_logger(), "failed to create publisher for can id %x", frame.can_id);
            return;
        }
        publish_frame(it->second, frame, meta);
    }
};

}

#endif // GUPPY_CAN_CAN_TRANSCEIVER
