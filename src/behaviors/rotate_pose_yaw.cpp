#include "tennis_demo_behaviorized/behaviors/rotate_pose_yaw.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace tennis_demo_behaviorized
{

RotatePoseYaw::RotatePoseYaw(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList RotatePoseYaw::providedPorts()
{
    return {
        // Input pose (SharedPtr so it matches your existing WalkToPose usage)
        BT::InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("in_pose"),

        // Yaw delta in degrees
        BT::InputPort<double>("delta_deg"),

        // Output pose (can be the same blackboard key as in_pose)
        BT::OutputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("out_pose")
    };
}

BT::NodeStatus RotatePoseYaw::tick()
{
    // --- Get input pose ---
    auto in_pose_exp = getInput<geometry_msgs::msg::PoseStamped::SharedPtr>("in_pose");
    if (!in_pose_exp) {
        throw BT::RuntimeError(
            "RotatePoseYaw: missing input [in_pose]: ", in_pose_exp.error());
    }

    auto in_pose_ptr = in_pose_exp.value();
    if (!in_pose_ptr) {
        throw BT::RuntimeError("RotatePoseYaw: [in_pose] is a null SharedPtr");
    }

    // --- Get delta in degrees ---
    auto delta_deg_exp = getInput<double>("delta_deg");
    if (!delta_deg_exp) {
        throw BT::RuntimeError(
            "RotatePoseYaw: missing input [delta_deg]: ", delta_deg_exp.error());
    }
    double delta_deg = delta_deg_exp.value();

    // Convert orientation to tf2::Quaternion
    const auto& q_in_msg = in_pose_ptr->pose.orientation;
    tf2::Quaternion q_in(q_in_msg.x, q_in_msg.y, q_in_msg.z, q_in_msg.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q_in).getRPY(roll, pitch, yaw);

    // Add delta yaw
    const double delta_rad = delta_deg * M_PI / 180.0;
    const double new_yaw = yaw + delta_rad;

    tf2::Quaternion q_new;
    q_new.setRPY(roll, pitch, new_yaw);
    q_new.normalize();

    // Write back orientation (in-place)
    auto out_pose_ptr = in_pose_ptr;  // default: in-place modify
    out_pose_ptr->pose.orientation.x = q_new.x();
    out_pose_ptr->pose.orientation.y = q_new.y();
    out_pose_ptr->pose.orientation.z = q_new.z();
    out_pose_ptr->pose.orientation.w = q_new.w();

    // Expose as output (can be same key as input in XML)
    setOutput("out_pose", out_pose_ptr);

    return BT::NodeStatus::SUCCESS;
}

}  // namespace tennis_demo_behaviorized