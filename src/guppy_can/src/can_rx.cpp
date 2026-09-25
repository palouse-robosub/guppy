#include "guppy_msgs/msg/can_frame.hpp"
#include "guppy_util/can.hpp"
#include "guppy_util/quality.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter_event_handler.hpp"

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

constexpr uint32_t CAN_ID_BIT_MASK = 1u << 10;

using namespace std::chrono_literals;

class CanRx : public rclcpp::Node {
private:
    using publisher_shared_ptr = std::shared_ptr<rclcpp::Publisher<guppy_msgs::msg::CanFrame>>;

    Socket                                            socket_;
    const rclcpp::TimerBase::SharedPtr                timer_;
    std::unordered_map<canid_t, publisher_shared_ptr> publishers_;
    std::thread                                       can_thread_;
    std::atomic<bool>                                 running_;

    const std::shared_ptr<rclcpp::ParameterEventHandler>   parameter_sub_;
    std::shared_ptr<const rclcpp::ParameterCallbackHandle> callback_handle_;
public:
    CanRx() :
        Node("can_rx"), socket_("can0", this->get_logger()),
        parameter_sub_(std::make_shared<rclcpp::ParameterEventHandler>(this)) {
        const auto interface = this->declare_parameter("interface", "can0");
        const auto callback  = [this](const rclcpp::Parameter& parameter) {
            socket_ = Socket(parameter.as_string(), this->get_logger());
            RCLCPP_DEBUG(
                this->get_logger(), "Received an update to parameter \"%s\" of type %s: \"%s\"",
                parameter.get_name().c_str(), parameter.get_type_name().c_str(),
                parameter.as_string().c_str()
            );
        };
        callback_handle_ = parameter_sub_->add_parameter_callback(interface, callback);
        running_.store(true);
        can_thread_ = std::thread(&CanRx::can_loop, this);
    }

    ~CanRx() {
        running_.store(false);
        if (can_thread_.joinable())
            can_thread_.join();
    }
private:
    // publishes data from can frame to ros topic
    void publish_bytes(canid_t id, const uint8_t data[], size_t len) const {
        guppy_msgs::msg::CanFrame frame;
        frame.can_id = id;
        frame.len    = len;
        frame.data.assign(data, data + len);
        frame.stamp   = this->now();
        const auto it = publishers_.find(id);
        it->second->publish(frame);
        RCLCPP_DEBUG(this->get_logger(), "0x%03X [%lu] ", id, len);
        for (size_t i = 0; i < len; i++)
            RCLCPP_DEBUG(this->get_logger(), "%02X ", data[i]);
        RCLCPP_DEBUG(this->get_logger(), "\r\n");
    }

    // creates a ros publisher for a given can id
    publisher_shared_ptr create_id_publisher(canid_t id) {
        static constexpr const auto base = "/can/id_0x";
        std::stringstream           stream;
        stream << base << std::hex << id;
        const auto topic_str = stream.str();
        const auto publisher =
            this->create_publisher<guppy_msgs::msg::CanFrame>(topic_str, quality::volatile_profile);
        publishers_.insert({id, publisher});
        RCLCPP_INFO(this->get_logger(), "created publisher %s\n", topic_str.c_str());
        return publisher;
    }

    void can_loop() {
        while (running_.load()) {
            can_frame frame{};
            if (!socket_.read(&frame)) {
                RCLCPP_WARN(this->get_logger(), "Reading from CAN frame failed!");
                continue;
            }
            // do not publish to ros if msb is 1
            if (frame.can_id & CAN_ID_BIT_MASK)
                continue;
            auto it = publishers_.find(frame.can_id);
            if (it != publishers_.end())
                if (create_id_publisher(frame.can_id))
                    publish_bytes(frame.can_id, frame.data, frame.len);
                else
                    RCLCPP_ERROR(
                        this->get_logger(), "Failed to create publisher for CAN ID %x", frame.can_id
                    );
            else
                publish_bytes(frame.can_id, frame.data, frame.len);
        }
    }
};

int main(const int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<CanRx>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
