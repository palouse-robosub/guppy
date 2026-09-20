#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <unordered_map>

#include "guppy_msgs/msg/can_frame.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter_event_handler.hpp"
#include "rclcpp/executors.hpp"

constexpr uint32_t CAN_ID_BIT_MASK = 1u << 10;

using namespace std::chrono_literals;

class CanRx : public rclcpp::Node {
public:
    static inline const auto reliable_profile = rclcpp::QoS(10).reliable();
    static inline const auto volatile_profile = rclcpp::QoS(10).best_effort().durability_volatile();
private:
    using publisher_shared_ptr = rclcpp::Publisher<guppy_msgs::msg::CanFrame>::SharedPtr;

    int                          socket_ = -1;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unordered_map<canid_t, publisher_shared_ptr>
                      publishers_;
    std::thread       can_thread_;
    std::atomic<bool> running_;

    std::shared_ptr<rclcpp::ParameterEventHandler>   parameter_sub_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> callback_handle_;
public:
    CanRx() : Node("can_rx") {
        this->declare_parameter("interface", "can0");
        initialize_socket();
        parameter_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        auto callback = [this](const rclcpp::Parameter& parameter) {
            initialize_socket();
            RCLCPP_DEBUG(
                this->get_logger(),
                "Received an update to parameter \"%s\" of type %s: \"%s\"",
                parameter.get_name().c_str(), parameter.get_type_name().c_str(),
                parameter.as_string().c_str()
            );
        };
        callback_handle_ = parameter_sub_->add_parameter_callback("interface", callback);
        running_.store(true);
        can_thread_ = std::thread(&CanRx::can_loop, this);
    }

    ~CanRx() {
        running_.store(false);
        if (socket_ >= 0)
            close(socket_);
        if (can_thread_.joinable())
            can_thread_.join();
    }
private:
    void initialize_socket() {
        this->socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (this->socket_ < 0)
            RCLCPP_FATAL(this->get_logger(), "Failed to create CAN socket!");
        auto interface = this->get_parameter("interface").as_string().c_str();
        ifreq request{};
        std::strcpy(request.ifr_name, interface);
        if (ioctl(socket_, SIOCGIFINDEX, &request) < 0)
            RCLCPP_FATAL(this->get_logger(), "SIOCGIFINDEX");
        sockaddr_can address{};
        address.can_family = AF_CAN, address.can_ifindex = request.ifr_ifindex;
        if (bind(socket_, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) < 0)
            RCLCPP_FATAL(this->get_logger(), "Failed to bind address to socket!");
    }

    // publishes data from can frame to ros topic
    void publish_bytes(canid_t id, const __u8 data[], int len) const {
        guppy_msgs::msg::CanFrame frame;
        frame.can_id = id;
        frame.len    = len;
        frame.data.assign(data, data + len);
        frame.stamp = this->now();
        auto it = publishers_.find(id);
        it->second->publish(frame);
        RCLCPP_DEBUG(this->get_logger(), "0x%03X [%d] ", id, len);
        for (int i = 0; i < len; i++)
            RCLCPP_DEBUG(this->get_logger(), "%02X ", data[i]);
        RCLCPP_DEBUG(this->get_logger(), "\r\n");
    }

    // creates a ros publisher for a given can id
    publisher_shared_ptr create_id_publisher(canid_t id) {
        static constexpr const auto base = "/can/id_0x";
        std::stringstream stream;
        stream << base << std::hex << id;
        auto topic_str = stream.str();
        auto publisher = this->create_publisher<guppy_msgs::msg::CanFrame>(topic_str, volatile_profile);
        publishers_.insert({id, publisher});
        RCLCPP_INFO(this->get_logger(), "created publisher %s\n", topic_str.c_str());
        return publisher;
    }

    void can_loop() {
        while (running_.load()) {
            can_frame frame{};
            const int nbytes = read(socket_, &frame, sizeof(can_frame));
            // checks for read error
            if (nbytes < 0) {
                RCLCPP_WARN(this->get_logger(), "Reading from CAN frame failed");
                continue;
            }
            // do not publish to ros if msb is 1
            if (frame.can_id & CAN_ID_BIT_MASK)
                continue;
            auto it = publishers_.find(frame.can_id);
            if (it != publishers_.end()) {
                if (create_id_publisher(frame.can_id))
                    publish_bytes(frame.can_id, frame.data, frame.can_dlc);
                else
                    RCLCPP_ERROR(this->get_logger(), "Failed to create publisher for CAN ID %x", frame.can_id);
            } else
                publish_bytes(frame.can_id, frame.data, frame.can_dlc);
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
