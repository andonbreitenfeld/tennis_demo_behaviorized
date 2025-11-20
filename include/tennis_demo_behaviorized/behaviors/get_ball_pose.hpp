#pragma once

#include <thread>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>

namespace tennis_demo
{

class GetBallPose : public BT::SyncActionNode
{
public:
    GetBallPose(const std::string& name, const BT::NodeConfig& config);
    ~GetBallPose();

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("ball_pose")
        };
    }

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ball_pose_sub_;
    std::optional<std::shared_ptr<geometry_msgs::msg::PoseStamped>> ball_pose_;
    std::thread spin_thread_;

    void ballPoseCallback(geometry_msgs::msg::PoseStamped::UniquePtr msg);
};

} // namespace tennis_demo