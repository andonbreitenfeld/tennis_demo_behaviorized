#pragma once

#include <thread>
#include <optional>
#include <memory>
#include <mutex>
#include <future> 

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// from original pick ball node
// #include <chrono>
// #include <cmath>
// #include <iostream>
// #include <atomic>
// #include <geometry_msgs/msg/pose.hpp>
// #include <moveit_msgs/msg/robot_trajectory.hpp>
// #include <tf2/LinearMath/Quaternion.h>


namespace tennis_demo
{

class DropBall : public BT::StatefulActionNode
{
public:
    DropBall(const std::string& name, const BT::NodeConfig& config);

    ~DropBall() override;

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;


private:
    // MoveIt
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ROS
    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::MultiThreadedExecutor executor_;
    std::thread spin_thread_;
    std::future<bool> future_;


    // helper functions and main logic from original node
    bool executePickAndPlace(const geometry_msgs::msg::PoseStamped& ball_pose);
    geometry_msgs::msg::Pose rotatePoseRoll90(const geometry_msgs::msg::Pose &pose_body);
    void moveToPose(const geometry_msgs::msg::Pose &target_pose, const std::string &pose_name);

};

} // namespace tennis_demo