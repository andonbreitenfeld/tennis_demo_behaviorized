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
  using namespace BT;

  return {
    InputPort<std::string>("nav_frame"),
    InputPort<std::string>("robot_frame"),
    InputPort<double>("standoff"),
    InputPort<geometry_msgs::msg::PoseStamped>("ball_pose"),
    OutputPort<geometry_msgs::msg::PoseStamped>("ball_nav_pose")
  };
}

BT::NodeStatus ComputeBallApproachGoal::tick()
{
  if (!rclcpp::ok()) {
    return BT::NodeStatus::FAILURE;
  }

  // Process TF subscriptions
  rclcpp::spin_some(node_);

  // Read inputs
  auto nav_frame_res   = getInput<std::string>("nav_frame");
  auto robot_frame_res = getInput<std::string>("robot_frame");
  auto standoff_res    = getInput<double>("standoff");
  auto ball_pose_res   = getInput<geometry_msgs::msg::PoseStamped>("ball_pose");

  if (!nav_frame_res || !robot_frame_res || !standoff_res || !ball_pose_res) {
    RCLCPP_WARN(node_->get_logger(), "ComputeBallApproachGoal: missing required input port");
    return BT::NodeStatus::FAILURE;
  }

  const std::string nav_frame   = nav_frame_res.value();
  const std::string robot_frame = robot_frame_res.value();
  const double standoff         = standoff_res.value();
  geometry_msgs::msg::PoseStamped ball_pose = ball_pose_res.value();

  // Ensure ball pose is in nav_frame
  geometry_msgs::msg::PoseStamped ball_in_nav;

  try {
    if (ball_pose.header.frame_id.empty() || ball_pose.header.frame_id == nav_frame) {
      ball_in_nav = ball_pose;
      ball_in_nav.header.frame_id = nav_frame;
    } else {
      auto tf_ball =
        tf_buffer_->lookupTransform(
          nav_frame,
          ball_pose.header.frame_id,
          tf2::TimePointZero,
          tf2::durationFromSec(0.2));

      tf2::doTransform(ball_pose, ball_in_nav, tf_ball);
    }
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(),
                "ComputeBallApproachGoal: failed to transform ball pose to %s: %s",
                nav_frame.c_str(), ex.what());
    return BT::NodeStatus::FAILURE;
  }

  const double ball_x = ball_in_nav.pose.position.x;
  const double ball_y = ball_in_nav.pose.position.y;

  // Get robot pose in nav_frame
  geometry_msgs::msg::TransformStamped tf_robot;
  rclcpp::Time now = node_->get_clock()->now();

  try {
    tf_robot = tf_buffer_->lookupTransform(
      nav_frame,
      robot_frame,
      now,
      rclcpp::Duration::from_seconds(0.2));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(),
                "ComputeBallApproachGoal: failed to lookup tf %s -> %s: %s",
                nav_frame.c_str(), robot_frame.c_str(), ex.what());
    return BT::NodeStatus::FAILURE;
  }

  const double robot_x = tf_robot.transform.translation.x;
  const double robot_y = tf_robot.transform.translation.y;

  // Direction from robot -> ball
  const double dx = ball_x - robot_x;
  const double dy = ball_y - robot_y;
  const double dist = std::hypot(dx, dy);

  if (dist < 1e-3) {
    RCLCPP_WARN(node_->get_logger(),
                "ComputeBallApproachGoal: robot and ball are too close to compute approach");
    return BT::NodeStatus::FAILURE;
  }

  const double ux = dx / dist;
  const double uy = dy / dist;

  // Goal is standoff meters "behind" the ball along robot->ball direction
  const double goal_x = ball_x - ux * standoff;
  const double goal_y = ball_y - uy * standoff;

  // Face the ball: orientation along (ux, uy)
  const double yaw = std::atan2(uy, ux);
  const double qz = std::sin(yaw / 2.0);
  const double qw = std::cos(yaw / 2.0);

  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = now;
  goal.header.frame_id = nav_frame;

  goal.pose.position.x = goal_x;
  goal.pose.position.y = goal_y;
  goal.pose.position.z = 0.0;

  goal.pose.orientation.x = 0.0;
  goal.pose.orientation.y = 0.0;
  goal.pose.orientation.z = qz;
  goal.pose.orientation.w = qw;

  setOutput("ball_nav_pose", goal);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace tennis_demo_behaviorized
