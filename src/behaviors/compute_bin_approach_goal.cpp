#include "tennis_demo_behaviorized/behaviors/compute_bin_approach_goal.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

ComputeBinApproachGoal::ComputeBinApproachGoal(
    const std::string& name,
    const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList ComputeBinApproachGoal::providedPorts()
{
  return {
    BT::InputPort<PosePtr>("tag_pose"),
    BT::InputPort<double>("standoff"),
    BT::OutputPort<PosePtr>("bin_nav_pose")
  };
}

BT::NodeStatus ComputeBinApproachGoal::tick()
{
  PosePtr tag_pose_ptr;
  if (!getInput<PosePtr>("tag_pose", tag_pose_ptr))
  {
    RCLCPP_ERROR(rclcpp::get_logger("ComputeBinApproachGoal"), 
      "tag_pose missing or wrong type");
    return BT::NodeStatus::FAILURE;
  }

  if (!tag_pose_ptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ComputeBinApproachGoal"),
      "tag_pose pointer is null");
    return BT::NodeStatus::FAILURE;
  }

  double standoff = 1.3;
  getInput<double>("standoff", standoff);

  auto tag = tag_pose_ptr->pose;

  double qx = tag.orientation.x;
  double qy = tag.orientation.y;
  double qz = tag.orientation.z;
  double qw = tag.orientation.w;

  double zmx = 2.0 * (qx * qz + qy * qw);
  double zmy = 2.0 * (qy * qz - qx * qw);

  double norm_xy = std::hypot(zmx, zmy);
  if (norm_xy < 1e-6)
    return BT::NodeStatus::FAILURE;

  zmx /= norm_xy;
  zmy /= norm_xy;

  auto nav_pose = std::make_shared<PoseStamped>();
  *nav_pose = *tag_pose_ptr; // copy header

  nav_pose->pose.position.x = tag.position.x + zmx * standoff;
  nav_pose->pose.position.y = tag.position.y + zmy * standoff;
  nav_pose->pose.position.z = 0.0;

  double yaw = std::atan2(-zmy, -zmx);
  nav_pose->pose.orientation.x = 0.0;
  nav_pose->pose.orientation.y = 0.0;
  nav_pose->pose.orientation.z = std::sin(yaw * 0.5);
  nav_pose->pose.orientation.w = std::cos(yaw * 0.5);

  setOutput("bin_nav_pose", nav_pose);

  RCLCPP_INFO(rclcpp::get_logger("ComputeBinApproachGoal"),
    "bin_nav_pose successfully written to blackboard!");

  return BT::NodeStatus::SUCCESS;
}
