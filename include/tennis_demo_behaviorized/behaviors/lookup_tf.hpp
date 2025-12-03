#pragma once

#include <behaviortree_cpp/action_node.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>
#include <string>
#include <chrono>
#include <memory>

namespace chair_manipulation
{

class LookupTF : public BT::StatefulActionNode, public rclcpp::Node
{
public:
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using PosePtr     = std::shared_ptr<PoseStamped>;

  LookupTF(const std::string &name, const BT::NodeConfig &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string ref_frame_;
  std::string target_frame_;
  double timeout_secs_ = 1.0;
  double tf_lookup_timeout_secs_ = 0.1;

  std::chrono::steady_clock::time_point start_time_;
};

} // namespace chair_manipulation
