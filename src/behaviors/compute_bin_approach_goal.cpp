#include "tennis_demo_behaviorized/behaviors/compute_bin_approach_goal.hpp"

#include <cmath>

namespace tennis_demo_behaviorized
{

ComputeBinApproachGoal::ComputeBinApproachGoal(
    const std::string& name,
    const BT::NodeConfig& config,
    const rclcpp::Node::SharedPtr& node)
  : BT::SyncActionNode(name, config),
    node_(node)
{
}

BT::PortsList ComputeBinApproachGoal::providedPorts()
{
  return {
    BT::InputPort<PosePtr>("tag_pose"),
    BT::InputPort<double>("standoff"),
    BT::OutputPort<PosePtr>("bin_nav_pose"),
    BT::InputPort<std::string>("bin_frame_id", "bin_nav_goal")
  };
}

BT::NodeStatus ComputeBinApproachGoal::tick()
{
  // Get inputs
  PosePtr tag_pose_ptr;
  if (!getInput("tag_pose", tag_pose_ptr) || !tag_pose_ptr) {
    RCLCPP_ERROR(node_->get_logger(), "tag_pose missing or null");
    return BT::NodeStatus::FAILURE;
  }

  double standoff = 1.7;
  getInput("standoff", standoff);

  const auto& tag = tag_pose_ptr->pose;

  // Extract tag's +Z axis direction projected to XY plane
  const double qx = tag.orientation.x;
  const double qy = tag.orientation.y;
  const double qz = tag.orientation.z;
  const double qw = tag.orientation.w;

  double zmx = 2.0 * (qx * qz + qy * qw);
  double zmy = 2.0 * (qy * qz - qx * qw);
  double norm_xy = std::hypot(zmx, zmy);

  if (norm_xy < 1e-6) {
    RCLCPP_ERROR(node_->get_logger(), "Tag Z axis nearly vertical");
    return BT::NodeStatus::FAILURE;
  }

  zmx /= norm_xy;
  zmy /= norm_xy;

  // Place goal in front of tag at standoff distance, facing toward tag
  auto nav_pose = std::make_shared<PoseStamped>();
  *nav_pose = *tag_pose_ptr;

  nav_pose->pose.position.x = tag.position.x + zmx * standoff;
  nav_pose->pose.position.y = tag.position.y + zmy * standoff;
  nav_pose->pose.position.z = 0.0;

  const double yaw = std::atan2(-zmy, -zmx);
  nav_pose->pose.orientation.x = 0.0;
  nav_pose->pose.orientation.y = 0.0;
  nav_pose->pose.orientation.z = std::sin(yaw * 0.5);
  nav_pose->pose.orientation.w = std::cos(yaw * 0.5);

  setOutput("bin_nav_pose", nav_pose);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace tennis_demo_behaviorized