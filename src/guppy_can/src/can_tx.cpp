#include "guppy_msgs/srv/send_can.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/executors.hpp"

#include <linux/can.h>
#include <linux/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

class CanTx : public rclcpp::Node {
public:
    static inline const auto reliable_profile = rclcpp::QoS(10).reliable();
    static inline const auto volatile_profile = rclcpp::QoS(10).best_effort().durability_volatile();
private:
    int socket_ = -1;
    rclcpp::Service<guppy_msgs::srv::SendCan>::SharedPtr send_service_;
public:
    CanTx() : Node("can_tx") {
        initialize_socket();

        this->send_service_ = this->create_service<guppy_msgs::srv::SendCan>(
            "can_tx",
            [this](
                const std::shared_ptr<guppy_msgs::srv::SendCan::Request>& request,
                const std::shared_ptr<guppy_msgs::srv::SendCan::Response>& response
            ) {
                this->send(request, response);
            },
            volatile_profile
        );

        RCLCPP_INFO(this->get_logger(), "CAN TX service ready.");
    }

    ~CanTx() {
        if (socket_ > 0)
            close(socket_);
    }
private:
    void initialize_socket() {
        this->socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (this->socket_ < 0)
            RCLCPP_FATAL(this->get_logger(), "Failed to create CAN socket!");
        ifreq request{};
        std::strncpy(request.ifr_name, "can0", IFNAMSIZ);
        if (ioctl(socket_, SIOCGIFINDEX, &request) < 0)
            RCLCPP_FATAL(this->get_logger(), "SIOCGIFINDEX");
        sockaddr_can address{};
        address.can_family = AF_CAN, address.can_ifindex = request.ifr_ifindex;
        if (bind(socket_, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) < 0)
            RCLCPP_FATAL(this->get_logger(), "Failed to bind address to socket!");
    }

    void send(
        const std::shared_ptr<guppy_msgs::srv::SendCan::Request>&  request,
        const std::shared_ptr<guppy_msgs::srv::SendCan::Response>& response
    ) {
        can_frame frame{};
        frame.can_id = request->id, frame.can_dlc = static_cast<__u8>(request->data.size());

        if (frame.can_dlc > 8) {
            RCLCPP_ERROR(this->get_logger(), "CAN payload too large (%zu bytes)", request->data.size());
            response->written = -1;
            return;
        }

        std::memcpy(frame.data, request->data.data(), frame.can_dlc);

        auto bytes        = write(socket_, &frame, sizeof(frame));
        response->written = bytes;

        if (bytes < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send message over CAN!");
            response->written = -1;
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    const auto transmit_node = std::make_shared<CanTx>();
    rclcpp::spin(transmit_node);
    rclcpp::shutdown();
    return 0;
}
