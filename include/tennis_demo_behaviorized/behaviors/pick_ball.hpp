#pragma once

#include <thread>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>

// from original pick ball node
#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/srv/trigger.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


namespace tennis_demo
{

class PickBall : public BT::StatefulActionNode
{
public:
    PickBall(const std::string& name, const BT::NodeConfig& config)
            : StatefulActionNode(name, config);

    ~PickBall();

    static BT::PortsList providedPorts();
    bool executePickAndPlace();
    geometry_msgs::msg::Pose rotatePoseRoll90(const geometry_msgs::msg::Pose &pose_body);

    BT::NodeStatus tick() override;


private:
    // from original pick ball node
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // from nav nodes
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr bin_pose_sub_;
    std::optional<std::shared_ptr<geometry_msgs::msg::PoseStamped>> bin_pose_;
    std::thread spin_thread_;
    void binPoseCallback(geometry_msgs::msg::PoseStamped::UniquePtr msg);

    // from original node
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void moveToPose(const geometry_msgs::msg::Pose &target_pose, const std::string &pose_name);

};

} // namespace tennis_demo