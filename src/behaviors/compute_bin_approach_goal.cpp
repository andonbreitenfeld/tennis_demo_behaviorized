#include "tennis_demo_behaviorized/behaviors/compute_bin_approach_goal.hpp"

#include <cmath>

namespace tennis_demo_behaviorized
{

ComputeBinApproachGoal::ComputeBinApproachGoal(
    const std::string & name,
    const BT::NodeConfig & config,
    const rclcpp::Node::SharedPtr & node)
: BT::SyncActionNode(name, config),
  node_(node),
  tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(node_))
{
}

BT::PortsList ComputeBinApproachGoal::providedPorts()
{
  return {
    BT::InputPort<PosePtr>("tag_pose"),
    BT::InputPort<double>("standoff"),
    BT::OutputPort<PosePtr>("bin_nav_pose"),
    // Optional TF frame name for the goal
    BT::InputPort<std::string>("bin_frame_id", std::string("bin_nav_goal"))
  };
}

BT::NodeStatus ComputeBinApproachGoal::tick()
{
  // --- 1. Get inputs from blackboard ---

  PosePtr tag_pose_ptr;
  if (!getInput<PosePtr>("tag_pose", tag_pose_ptr))
  {
    RCLCPP_ERROR(node_->get_logger(),
      "[ComputeBinApproachGoal] tag_pose missing or wrong type");
    return BT::NodeStatus::FAILURE;
  }

  if (!tag_pose_ptr)
  {
    RCLCPP_ERROR(node_->get_logger(),
      "[ComputeBinApproachGoal] tag_pose pointer is null");
    return BT::NodeStatus::FAILURE;
  }

  double standoff = 1.7;            // default standoff
  (void)getInput<double>("standoff", standoff);

  const auto & tag = tag_pose_ptr->pose;

  // --- 2. Compute approach direction from tag orientation (+Z projected in XY) ---

  const double qx = tag.orientation.x;
  const double qy = tag.orientation.y;
  const double qz = tag.orientation.z;
  const double qw = tag.orientation.w;

  // Projection of tag's +Z axis into XY plane in map frame
  double zmx = 2.0 * (qx * qz + qy * qw);
  double zmy = 2.0 * (qy * qz - qx * qw);

  double norm_xy = std::hypot(zmx, zmy);
  if (norm_xy < 1e-6)
  {
    RCLCPP_ERROR(node_->get_logger(),
      "[ComputeBinApproachGoal] projected Z axis too small (norm_xy < 1e-6)");
    return BT::NodeStatus::FAILURE;
  }

  zmx /= norm_xy;
  zmy /= norm_xy;

  // --- 3. Build nav pose at standoff distance, facing the tag ---

  auto nav_pose = std::make_shared<PoseStamped>();
  *nav_pose = *tag_pose_ptr;        // copy header & base pose

  // Position: move out along tag's +Z (projected in XY)
  nav_pose->pose.position.x = tag.position.x + zmx * standoff;
  nav_pose->pose.position.y = tag.position.y + zmy * standoff;
  nav_pose->pose.position.z = 0.0;  // usually on the ground plane

  // Orientation: face back toward the tag
  const double yaw = std::atan2(-zmy, -zmx);
  nav_pose->pose.orientation.x = 0.0;
  nav_pose->pose.orientation.y = 0.0;
  nav_pose->pose.orientation.z = std::sin(yaw * 0.5);
  nav_pose->pose.orientation.w = std::cos(yaw * 0.5);

  // --- 4. Write to blackboard ---

  setOutput("bin_nav_pose", nav_pose);

  // --- 5. Publish TF for visualization / other consumers ---

  std::string child_frame = "bin_nav_goal";
  (void)getInput<std::string>("bin_frame_id", child_frame);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = nav_pose->header.frame_id;  // e.g. "spot_nav/map"
  tf.header.stamp    = node_->now();               // fresh timestamp each tick
  tf.child_frame_id  = child_frame;

  tf.transform.translation.x = nav_pose->pose.position.x;
  tf.transform.translation.y = nav_pose->pose.position.y;
  tf.transform.translation.z = nav_pose->pose.position.z;
  tf.transform.rotation      = nav_pose->pose.orientation;

  tf_broadcaster_->sendTransform(tf);

  RCLCPP_INFO(node_->get_logger(),
    "[ComputeBinApproachGoal] bin_nav_pose written to blackboard and TF (%s in %s)",
    tf.child_frame_id.c_str(), tf.header.frame_id.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace tennis_demo_behaviorized