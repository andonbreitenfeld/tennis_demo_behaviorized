#ifndef LOOKUP_TF_HPP_
#define LOOKUP_TF_HPP_

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <ros_cpp_util/ros_cpp_util.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <string>
#include <memory>
#include <Eigen/Core>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace chair_manipulation
{

class LookupTF : public BT::StatefulActionNode, rclcpp::Node
{

  public:
    /**
     * @brief Any TreeNode with ports must have this constructor signature
     */
    LookupTF(const std::string &name, const BT::NodeConfig &config);

    /**
     * @brief Mandatory method
     */
    static BT::PortsList providedPorts();

    /**
     * @brief Invoked once at the beginning
     */
    BT::NodeStatus onStart() override;

    /**
     * @brief If onStart() returned RUNNING, this method is constantly called until it returns something other than
     * RUNNING
     */
    BT::NodeStatus onRunning() override;

    /**
     * @brief Callback to execute if the action was aborted by another node
     */
    void onHalted() override;

  private:
  std::unique_ptr<tf2_ros::Buffer>
      tf_buffer_; ///< TF2 buffer for transformation lookup
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
	std::string ref_frame_;
	std::string target_frame_;
	double timeout_secs_;
double tf_lookup_timeout_secs_ = 0.5;//secs
	Eigen::Isometry3d pose_;
	std::chrono::steady_clock::time_point start_time_;
};

} // namespace chair_manipulation

#endif