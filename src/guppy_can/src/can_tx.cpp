#include "guppy_msgs/srv/send_can.hpp"
#include "guppy_util/can.hpp"
#include "guppy_util/quality.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"

class CanTx : public rclcpp::Node {
private:
    const Socket                                                           socket_;
    const std::shared_ptr<const rclcpp::Service<guppy_msgs::srv::SendCan>> send_service_;
public:
    CanTx() :
        Node("can_tx"), socket_("can0", this->get_logger()),
        send_service_(this->create_service<guppy_msgs::srv::SendCan>(
            "can_tx",
            [this](
                const std::shared_ptr<guppy_msgs::srv::SendCan::Request>&  request,
                const std::shared_ptr<guppy_msgs::srv::SendCan::Response>& response
            ) { this->send(request, response); },
            quality::volatile_profile
        )) {
        RCLCPP_INFO(this->get_logger(), "CAN TX service ready.");
    }
private:
    void send(
        const std::shared_ptr<const guppy_msgs::srv::SendCan::Request>& request,
        const std::shared_ptr<guppy_msgs::srv::SendCan::Response>&      response
    ) {
        can_frame frame{};
        frame.can_id = request->id;
        auto length  = static_cast<__u8>(request->data.size());
        if (length > 8) {
            RCLCPP_ERROR(
                this->get_logger(), "CAN payload too large (%zu bytes)", request->data.size()
            );
            response->success = false;
            return;
        }
        frame.len = length;
        std::memcpy(frame.data, request->data.data(), length);
        size_t bytes_written;
        if (!socket_.write(&frame, &bytes_written)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send message over CAN!");
            response->success = false;
            return;
        }
        response->written = static_cast<uint8_t>(bytes_written);
        response->success = true;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    const auto transmit_node = std::make_shared<CanTx>();
    rclcpp::spin(transmit_node);
    rclcpp::shutdown();
    return 0;
}
