#include "tennis_demo_behaviorized/behaviors/rotate_pose_yaw.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

namespace tennis_demo_behaviorized
{

RotatePoseYaw::RotatePoseYaw(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList RotatePoseYaw::providedPorts()
{
  return {
    BT::InputPort<PoseStampedPtr>("in_pose"),
    BT::InputPort<double>("delta_deg"),
    BT::OutputPort<PoseStampedPtr>("out_pose")
  };
}

BT::NodeStatus RotatePoseYaw::tick()
{
  PoseStampedPtr in_pose_ptr;
  double delta_deg;

  if (!getInput("in_pose", in_pose_ptr) || !in_pose_ptr ||
      !getInput("delta_deg", delta_deg)) {
    throw BT::RuntimeError("RotatePoseYaw: missing required inputs");
  }

  // Extract current orientation
  const auto& q_msg = in_pose_ptr->pose.orientation;
  tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);

  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // Apply yaw rotation
  const double delta_rad = delta_deg * M_PI / 180.0;
  tf2::Quaternion q_new;
  q_new.setRPY(roll, pitch, yaw + delta_rad);
  q_new.normalize();

  // Update orientation in-place
  in_pose_ptr->pose.orientation.x = q_new.x();
  in_pose_ptr->pose.orientation.y = q_new.y();
  in_pose_ptr->pose.orientation.z = q_new.z();
  in_pose_ptr->pose.orientation.w = q_new.w();

  setOutput("out_pose", in_pose_ptr);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace tennis_demo_behaviorized