#pragma once

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <memory>

namespace tennis_demo_behaviorized
{

class ComputeBallApproachGoal : public BT::SyncActionNode
{
public:
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using PosePtr = std::shared_ptr<PoseStamped>;

  ComputeBallApproachGoal(
      const std::string& name,
      const BT::NodeConfig& config,
      const rclcpp::Node::SharedPtr& node,
      const std::shared_ptr<tf2_ros::Buffer>& tf_buffer);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace tennis_demo_behaviorized