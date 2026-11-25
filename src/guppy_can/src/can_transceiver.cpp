#include "guppy_can/can_transceiver.hpp"

#include <filesystem>
#include <linux/can.h>

#include <exception>
#include <stdexcept>
#include <string_view>
#include <unordered_map>
#include <variant>
#include <string>
#include <utility>

#include <rclcpp/executors.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_resource.hpp>

#include "can_xml/xml_parse_extern.hpp"
#include "guppy_util/socket.hpp"
#include "guppy_util/can.hpp"

namespace guppy_can {

CANTransceiver::Registry CANTransceiver::Registry::load(const std::filesystem::path &file) {
    return Registry(can_xml::parse_xml(file));
}

CANTransceiver::Registry::Registry(const std::vector<can_xml::CANIDMeta> entries) : m_entries(std::move(entries)) {
    m_by_id.reserve(m_entries.size());
    m_by_name.reserve(m_entries.size());
    for (size_t i = 0; i < m_entries.size(); ++i) {
        const auto& entry = m_entries[i];
        if (auto [it, inserted] = m_by_id.emplace(entry.id.id, i); !inserted)
            throw std::runtime_error("duplicate can id!");
        if (auto [it, inserted] = m_by_name.emplace(entry.name, i); !inserted)
            throw std::runtime_error("duplicate can id name!");
    }
}

const can_xml::CANIDMeta* CANTransceiver::Registry::find(canid_t id) const {
    const auto it = m_by_id.find(id);
    return it == m_by_id.cend() ? nullptr : &m_entries[it->second];
}

const can_xml::CANIDMeta* CANTransceiver::Registry::find(const std::string& name) const {
    const auto it = m_by_name.find(name);
    return it == m_by_name.cend() ? nullptr : &m_entries[it->second];
}

CANTransceiver::CANTransceiver(std::unique_ptr<guppy_util::ISocket> socket, const rclcpp::NodeOptions& options) : rclcpp::Node("can_transceiver", options), m_socket(std::move(socket)) {
    load_registry();
    start_receiver_thread();
}

CANTransceiver::CANTransceiver(const rclcpp::NodeOptions& options) : rclcpp::Node("can_transceiver", options) {
    load_registry();
    declare_parameter<std::string>("interface", "can0");
    std::string interface;
    get_parameter("interface", interface);
    create_socket(interface);
    start_receiver_thread();
}

CANTransceiver::~CANTransceiver() {
    stop_receiver_thread();
}

void CANTransceiver::create_socket(std::string_view interface) {
    m_socket = std::make_unique<guppy_util::Socket>(interface);
}

void CANTransceiver::load_registry() {
    try {
        std::string content, prefix;
        auto found = ament_index_cpp::get_resource("can_ids", "guppy_can", content, &prefix);
        if (!found)
            RCLCPP_FATAL(get_logger(), "resource file not found!");
        std::filesystem::path resource_path = std::filesystem::path(prefix) / "share/ament_index/resource_index/can_ids/can_xml";
        m_registry = Registry::load(resource_path);
    } catch (const std::exception& exception) {
        RCLCPP_FATAL(get_logger(), "failed to create registry, %s", exception.what());
    }
}

void CANTransceiver::start_receiver_thread() {
    m_stop_thread = false;
    m_receiver_thread = std::thread(&CANTransceiver::receive, this);
}

void CANTransceiver::stop_receiver_thread() {
    m_stop_thread = true;
    m_socket->stop();
    if (m_receiver_thread.joinable())
        m_receiver_thread.join();
}

// creates a ros publisher for a given can id
CANTransceiver::publisher_shared_ptr CANTransceiver::create_id_publisher(const can_xml::CANIDMeta* meta, canid_t id) {
    std::string topic_str;
    if (meta)
        topic_str = guppy_util::format_can_topic(meta->name);
    else
        topic_str = guppy_util::format_can_topic(id);
    const auto publisher = create_publisher<guppy_msgs::msg::CANFrame>(topic_str, guppy_util::quality::volatile_profile);
    m_publishers.emplace(id, publisher);
    RCLCPP_INFO(get_logger(), "created publisher '%s'", topic_str.c_str());
    return publisher;
}

// receiving loop in another thread
void CANTransceiver::receive() {
    try {
        while(!m_stop_thread) {
            auto frame = m_socket->receive();
            if (!frame) {
                RCLCPP_ERROR(get_logger(), "reading from can frame failed!");
                continue;
            }
            if (const can_frame* classical_frame = std::get_if<can_frame>(&frame.value())) {
                process(*classical_frame);
            } else {
                const auto& flexible_frame = std::get<canfd_frame>(frame.value());
                process(flexible_frame);
            }
        }
    } catch (const std::exception& exception) {
        if (!m_stop_thread)
            RCLCPP_FATAL(get_logger(), "can receiver thread failed: %s", exception.what());
    }
}

void CANTransceiver::send(
    const std::shared_ptr<const send_can::Request>& request,
    const std::shared_ptr<send_can::Response>&      response
) {
#ifdef ENABLE_CAN_FD
    canfd_frame frame{};
#else
    can_frame frame{};
#endif
    auto* meta = m_registry.find(request->name);
    if (!meta) {
        RCLCPP_ERROR(get_logger(), "no corresponding id for name '%s'!", request->name.c_str());
        response->success = false;
        return;
    }
    frame.can_id = meta->id.id;
    auto length  = static_cast<uint8_t>(request->data.size());
#ifdef ENABLE_CAN_FD
    if (length > 64) {
#else
    if (length > 8) {
#endif
        RCLCPP_ERROR(get_logger(), "can payload too large! (%zu bytes)", request->data.size());
        response->success = false;
        return;
    }
    frame.len = length;
    std::memcpy(frame.data, request->data.data(), length);
    try {
        response->written = static_cast<uint8_t>(m_socket->send(frame));
    } catch (std::exception& exception) {
        RCLCPP_ERROR(get_logger(), "failed to send message: %s", exception.what());
        response->success = false;
        return;
    }
    response->success = true;
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(guppy_can::CANTransceiver)
