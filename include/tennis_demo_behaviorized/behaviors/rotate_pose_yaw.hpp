#pragma once

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace tennis_demo_behaviorized {

class RotatePoseYaw : public BT::SyncActionNode
{
public:
    RotatePoseYaw(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};

}  // namespace nrg_utility_behaviors