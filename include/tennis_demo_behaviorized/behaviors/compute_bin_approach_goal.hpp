#pragma once

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

namespace tennis_demo_behaviorized
{

class ComputeBinApproachGoal : public BT::SyncActionNode
{
public:
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using PosePtr = std::shared_ptr<PoseStamped>;

  ComputeBinApproachGoal(
      const std::string& name,
      const BT::NodeConfig& config,
      const rclcpp::Node::SharedPtr& node);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace tennis_demo_behaviorized