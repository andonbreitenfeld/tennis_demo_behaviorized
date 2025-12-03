#include "tennis_demo_behaviorized/behaviors/compute_bin_approach_goal.hpp"
#include <cmath>

ComputeBinApproachGoal::ComputeBinApproachGoal(
    const std::string& name,
    const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList ComputeBinApproachGoal::providedPorts()
{
  using PosePtr = std::shared_ptr<geometry_msgs::msg::PoseStamped>;

  return {
    BT::InputPort<PosePtr>("tag_pose"),
    BT::InputPort<double>("standoff", 1.2),
    BT::OutputPort<PosePtr>("bin_nav_pose")
  };
}

BT::NodeStatus ComputeBinApproachGoal::tick()
{
  using PosePtr = std::shared_ptr<geometry_msgs::msg::PoseStamped>;

  // -------------------------
  // Get inputs
  // -------------------------
  auto tag_opt = getInput<PosePtr>("tag_pose");
  if (!tag_opt || !tag_opt.value()) {
    throw BT::RuntimeError("ComputeBinApproachGoal: missing input [tag_pose]");
  }
  PosePtr tag_ptr = tag_opt.value();

  double standoff = 1.2;
  getInput<double>("standoff", standoff);

  const auto& tag = tag_ptr->pose;

  // -------------------------
  // Extract tag pose
  // -------------------------
  const double tag_x = tag.position.x;
  const double tag_y = tag.position.y;

  const double qx = tag.orientation.x;
  const double qy = tag.orientation.y;
  const double qz = tag.orientation.z;
  const double qw = tag.orientation.w;

  // Compute tag +Z in map frame (same as Python)
  double zmx = 2.0 * (qx * qz + qy * qw);
  double zmy = 2.0 * (qy * qz - qx * qw);

  double norm_xy = std::hypot(zmx, zmy);
  if (norm_xy < 1e-6) {
    // Too vertical — can't compute a horizontal approach direction
    return BT::NodeStatus::FAILURE;
  }

  zmx /= norm_xy;
  zmy /= norm_xy;

  // -------------------------
  // Compute approach position
  // -------------------------
  const double stand_x = tag_x + zmx * standoff;
  const double stand_y = tag_y + zmy * standoff;

  // Robot should face the tag: +X points opposite of Z_tag
  const double yaw = std::atan2(-zmy, -zmx);

  const double half_yaw = yaw * 0.5;
  const double qz_nav   = std::sin(half_yaw);
  const double qw_nav   = std::cos(half_yaw);

  // -------------------------
  // Build output PoseStamped
  // -------------------------
  PosePtr nav_goal = std::make_shared<geometry_msgs::msg::PoseStamped>();
  *nav_goal = *tag_ptr;  // copy header/frame_id

  nav_goal->pose.position.x = stand_x;
  nav_goal->pose.position.y = stand_y;
  nav_goal->pose.position.z = 0.0;

  nav_goal->pose.orientation.x = 0.0;
  nav_goal->pose.orientation.y = 0.0;
  nav_goal->pose.orientation.z = qz_nav;
  nav_goal->pose.orientation.w = qw_nav;

  // Write to blackboard
  setOutput<PosePtr>("bin_nav_pose", nav_goal);

  return BT::NodeStatus::SUCCESS;
}
