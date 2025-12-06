#pragma once

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <mutex>

namespace tennis_demo_behaviorized
{

class ComputeBallApproachGoal : public BT::SyncActionNode
{
public:
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using PosePtr     = std::shared_ptr<PoseStamped>;

  ComputeBallApproachGoal(const std::string& name,
                          const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  static std::once_flag init_flag_;
  static rclcpp::Node::SharedPtr node_;
  static std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  static std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace tennis_demo_behaviorized