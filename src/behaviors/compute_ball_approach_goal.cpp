#include "tennis_demo_behaviorized/behaviors/compute_ball_approach_goal.hpp"

#include <cmath>
#include <mutex>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace tennis_demo_behaviorized
{

std::once_flag ComputeBallApproachGoal::init_flag_;
rclcpp::Node::SharedPtr ComputeBallApproachGoal::node_;
std::shared_ptr<tf2_ros::Buffer> ComputeBallApproachGoal::tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> ComputeBallApproachGoal::tf_listener_;

ComputeBallApproachGoal::ComputeBallApproachGoal(
    const std::string& name,
    const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
  std::call_once(init_flag_, []() {
    node_ = rclcpp::Node::make_shared("compute_ball_approach_goal_bt_node");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  });
}

BT::PortsList ComputeBallApproachGoal::providedPorts()
{
  using PosePtr = ComputeBallApproachGoal::PosePtr;

  return {
    BT::InputPort<std::string>("nav_frame"),
    BT::InputPort<std::string>("robot_frame"),
    BT::InputPort<double>("standoff"),
    BT::InputPort<PosePtr>("ball_pose"),
    BT::OutputPort<PosePtr>("ball_nav_pose")
  };
}

BT::NodeStatus ComputeBallApproachGoal::tick()
{
  if (!rclcpp::ok()) {
    return BT::NodeStatus::FAILURE;
  }

  rclcpp::spin_some(node_);

  auto nav_frame_res   = getInput<std::string>("nav_frame");
  auto robot_frame_res = getInput<std::string>("robot_frame");
  auto standoff_res    = getInput<double>("standoff");
  auto ball_pose_res   = getInput<PosePtr>("ball_pose");

  // Static config must exist
  if (!nav_frame_res || !robot_frame_res || !standoff_res) {
    return BT::NodeStatus::FAILURE;
  }

  // If there is no ball pose yet, just fail quietly
  if (!ball_pose_res || !ball_pose_res.value()) {
    return BT::NodeStatus::FAILURE;
  }

  const std::string nav_frame   = nav_frame_res.value();
  const std::string robot_frame = robot_frame_res.value();
  const double standoff         = standoff_res.value();
  PosePtr ball_pose_ptr         = ball_pose_res.value();

  // Copy ball pose so we can transform it
  geometry_msgs::msg::PoseStamped ball_pose = *ball_pose_ptr;
  geometry_msgs::msg::PoseStamped ball_in_nav;

  // Ensure ball pose is in nav_frame
  try
  {
    if (ball_pose.header.frame_id.empty() ||
        ball_pose.header.frame_id == nav_frame)
    {
      ball_in_nav = ball_pose;
      ball_in_nav.header.frame_id = nav_frame;
    }
    else
    {
      auto tf_ball =
        tf_buffer_->lookupTransform(
          nav_frame,
          ball_pose.header.frame_id,
          tf2::TimePointZero,
          tf2::durationFromSec(0.2));

      tf2::doTransform(ball_pose, ball_in_nav, tf_ball);
    }
  }
  catch (const tf2::TransformException&)
  {
    // Transform not available yet → normal failure
    return BT::NodeStatus::FAILURE;
  }

  const double ball_x = ball_in_nav.pose.position.x;
  const double ball_y = ball_in_nav.pose.position.y;

  // Get robot pose in nav_frame
  geometry_msgs::msg::TransformStamped tf_robot;
  rclcpp::Time now = node_->get_clock()->now();

  try
  {
    tf_robot = tf_buffer_->lookupTransform(
      nav_frame,
      robot_frame,
      now,
      rclcpp::Duration::from_seconds(0.2));
  }
  catch (const tf2::TransformException&)
  {
    // TF not ready → normal failure
    return BT::NodeStatus::FAILURE;
  }

  const double robot_x = tf_robot.transform.translation.x;
  const double robot_y = tf_robot.transform.translation.y;

  // Direction from robot -> ball
  const double dx = ball_x - robot_x;
  const double dy = ball_y - robot_y;
  const double dist = std::hypot(dx, dy);

  if (dist < 1e-3) {
    // Degenerate geometry → fail
    return BT::NodeStatus::FAILURE;
  }

  const double ux = dx / dist;
  const double uy = dy / dist;

  // Goal is standoff meters behind the ball along robot->ball direction
  const double goal_x = ball_x - ux * standoff;
  const double goal_y = ball_y - uy * standoff;

  // Orient robot to face the ball
  const double yaw = std::atan2(uy, ux);
  const double qz  = std::sin(yaw / 2.0);
  const double qw  = std::cos(yaw / 2.0);

  PosePtr goal_ptr = std::make_shared<PoseStamped>();
  goal_ptr->header.stamp = now;
  goal_ptr->header.frame_id = nav_frame;

  goal_ptr->pose.position.x = goal_x;
  goal_ptr->pose.position.y = goal_y;
  goal_ptr->pose.position.z = 0.0;

  goal_ptr->pose.orientation.x = 0.0;
  goal_ptr->pose.orientation.y = 0.0;
  goal_ptr->pose.orientation.z = qz;
  goal_ptr->pose.orientation.w = qw;

  setOutput("ball_nav_pose", goal_ptr);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace tennis_demo_behaviorized