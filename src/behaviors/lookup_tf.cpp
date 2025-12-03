#include "tennis_demo_behaviorized/behaviors/lookup_tf.hpp"

namespace chair_manipulation
{

LookupTF::LookupTF(const std::string &name, const BT::NodeConfig &config)
  : BT::StatefulActionNode(name, config)
  , rclcpp::Node(name)
{
  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList LookupTF::providedPorts()
{
  return {
    BT::InputPort<std::string>("ref_frame"),
    BT::InputPort<std::string>("target_frame"),
    BT::InputPort<double>("timeout_secs"),
    BT::OutputPort<PosePtr>("pose")
  };
}

BT::NodeStatus LookupTF::onStart()
{
  if (!getInput<std::string>("ref_frame", ref_frame_))
  {
    throw BT::RuntimeError("LookupTF: missing required input [ref_frame]");
  }

  if (!getInput<std::string>("target_frame", target_frame_))
  {
    throw BT::RuntimeError("LookupTF: missing required input [target_frame]");
  }

  if (!getInput<double>("timeout_secs", timeout_secs_))
  {
    throw BT::RuntimeError("LookupTF: missing required input [timeout_secs]");
  }

  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LookupTF::onRunning()
{
  // Allow TF callbacks to process
  rclcpp::spin_some(this->get_node_base_interface());

  geometry_msgs::msg::TransformStamped tf_stamped;
  geometry_msgs::msg::PoseStamped pose_out;

  // Check timeout
  const auto   now     = std::chrono::steady_clock::now();
  const double elapsed = std::chrono::duration<double>(now - start_time_).count();

  if (elapsed > timeout_secs_)
  {
    RCLCPP_WARN(
        this->get_logger(),
        "TF lookup timed out after %.2f seconds (ref: '%s' to target: '%s')",
        timeout_secs_, ref_frame_.c_str(), target_frame_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  try
  {
    // Try to lookup the latest transform
    tf_stamped = tf_buffer_->lookupTransform(
        ref_frame_,             // target frame
        target_frame_,          // source frame
        tf2::TimePointZero,     // latest
        tf2::durationFromSec(tf_lookup_timeout_secs_));

    // Convert to PoseStamped
    pose_out.header = tf_stamped.header;
    pose_out.pose.position.x = tf_stamped.transform.translation.x;
    pose_out.pose.position.y = tf_stamped.transform.translation.y;
    pose_out.pose.position.z = tf_stamped.transform.translation.z;
    pose_out.pose.orientation = tf_stamped.transform.rotation;

    auto pose_ptr = std::make_shared<PoseStamped>(pose_out);

    // Output result and return success
    setOutput("pose", pose_ptr);

    RCLCPP_DEBUG(
        this->get_logger(),
        "LookupTF succeeded: %s -> %s  (pos: %.3f, %.3f, %.3f)",
        ref_frame_.c_str(), target_frame_.c_str(),
        pose_out.pose.position.x,
        pose_out.pose.position.y,
        pose_out.pose.position.z);

    return BT::NodeStatus::SUCCESS;
  }
  catch (const tf2::TransformException &)
  {
    // Not found yet — keep trying
    return BT::NodeStatus::RUNNING;
  }
}

void LookupTF::onHalted()
{
  // nothing to clean up
}

} // namespace chair_manipulation
