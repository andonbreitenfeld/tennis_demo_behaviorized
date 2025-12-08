#include "tennis_demo_behaviorized/behaviors/compute_ball_approach_goal.hpp"

#include <cmath>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace tennis_demo_behaviorized
{

ComputeBallApproachGoal::ComputeBallApproachGoal(
    const std::string& name,
    const BT::NodeConfig& config,
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<tf2_ros::Buffer>& tf_buffer)
  : BT::SyncActionNode(name, config),
    node_(node),
    tf_buffer_(tf_buffer)
{
}

BT::PortsList ComputeBallApproachGoal::providedPorts()
{
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
  rclcpp::spin_some(node_);

  // Get inputs
  PosePtr ball_pose_ptr;
  std::string nav_frame, robot_frame;
  double standoff;

  if (!getInput("ball_pose", ball_pose_ptr) || !ball_pose_ptr ||
      !getInput("nav_frame", nav_frame) ||
      !getInput("robot_frame", robot_frame) ||
      !getInput("standoff", standoff)) {
    return BT::NodeStatus::FAILURE;
  }

  const rclcpp::Time now = node_->now();
  geometry_msgs::msg::PoseStamped ball_pose = *ball_pose_ptr;
  geometry_msgs::msg::PoseStamped ball_in_nav;

  try {
    // Transform ball to nav frame
    if (ball_pose.header.frame_id.empty() || ball_pose.header.frame_id == nav_frame) {
      ball_in_nav = ball_pose;
      ball_in_nav.header.frame_id = nav_frame;
    } else {
      auto tf_ball = tf_buffer_->lookupTransform(
        nav_frame, ball_pose.header.frame_id,
        tf2::TimePointZero, tf2::durationFromSec(0.2));
      tf2::doTransform(ball_pose, ball_in_nav, tf_ball);
    }

    // Get robot position
    auto tf_robot = tf_buffer_->lookupTransform(
      nav_frame, robot_frame,
      now, rclcpp::Duration::from_seconds(0.2));

    const double ball_x = ball_in_nav.pose.position.x;
    const double ball_y = ball_in_nav.pose.position.y;
    const double robot_x = tf_robot.transform.translation.x;
    const double robot_y = tf_robot.transform.translation.y;

    // Compute direction from robot to ball
    const double dx = ball_x - robot_x;
    const double dy = ball_y - robot_y;
    const double dist = std::hypot(dx, dy);

    if (dist < 1e-3) {
      return BT::NodeStatus::FAILURE;
    }

    const double ux = dx / dist;
    const double uy = dy / dist;

    // Place goal behind ball at standoff distance, facing toward ball
    const double goal_x = ball_x - ux * standoff;
    const double goal_y = ball_y - uy * standoff;
    const double yaw = std::atan2(uy, ux);

    auto goal_pose = std::make_shared<PoseStamped>();
    goal_pose->header.stamp = now;
    goal_pose->header.frame_id = nav_frame;
    goal_pose->pose.position.x = goal_x;
    goal_pose->pose.position.y = goal_y;
    goal_pose->pose.position.z = 0.0;
    goal_pose->pose.orientation.x = 0.0;
    goal_pose->pose.orientation.y = 0.0;
    goal_pose->pose.orientation.z = std::sin(yaw * 0.5);
    goal_pose->pose.orientation.w = std::cos(yaw * 0.5);

    setOutput("ball_nav_pose", goal_pose);
    return BT::NodeStatus::SUCCESS;

  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(), "TF lookup failed: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace tennis_demo_behaviorized