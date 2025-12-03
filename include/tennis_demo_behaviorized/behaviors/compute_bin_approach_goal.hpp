#pragma once

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>

class ComputeBinApproachGoal : public BT::SyncActionNode
{
public:
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using PosePtr     = std::shared_ptr<PoseStamped>;

  ComputeBinApproachGoal(const std::string& name,
                         const BT::NodeConfig& config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};
