#pragma once

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>

namespace tennis_demo_behaviorized
{

class RotatePoseYaw : public BT::SyncActionNode
{
public:
  using PoseStampedPtr = std::shared_ptr<geometry_msgs::msg::PoseStamped>;

  RotatePoseYaw(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace tennis_demo_behaviorized