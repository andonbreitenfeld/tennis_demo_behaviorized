#pragma once

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>

namespace tennis_demo_behaviorized
{

class ComputeBinApproachGoal : public BT::SyncActionNode
{
public:
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using PosePtr     = std::shared_ptr<PoseStamped>;

  // Note: ctor takes a shared rclcpp::Node so we can log + broadcast TF
  ComputeBinApproachGoal(const std::string & name,
                         const BT::NodeConfig & config,
                         const rclcpp::Node::SharedPtr & node);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace tennis_demo_behaviorized
