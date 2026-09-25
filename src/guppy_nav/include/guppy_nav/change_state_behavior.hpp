#ifndef CHANGE_STATE_BEHAVIOR_HPP
#define CHANGE_STATE_BEHAVIOR_HPP

#include <behaviortree_ros2/bt_service_node.hpp>
#include <guppy_msgs/srv/change_state.hpp>

class ChangeStateBehavior : public BT::RosServiceNode<guppy_msgs::srv::ChangeState> {
public:
    ChangeStateBehavior(
        const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& parameters
    );
    static BT::PortsList providedPorts();
    bool                 setRequest(std::shared_ptr<Request>& request) override;
    BT::NodeStatus       onResponseReceived(const std::shared_ptr<Response>& response) override;
};

#endif
