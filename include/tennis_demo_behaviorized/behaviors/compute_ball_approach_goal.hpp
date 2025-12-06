#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace tennis_demo_behaviorized
{

class ComputeBallApproachGoal : public BT::SyncActionNode
{
public:
  ComputeBallApproachGoal(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  // Shared TF context for all instances
  static std::once_flag init_flag_;
  static rclcpp::Node::SharedPtr node_;
  static std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  static std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace tennis_demo_behaviorized
