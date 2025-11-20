#pragma once

#include <thread>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>

namespace tennis_demo
{

class GetBinPose : public BT::SyncActionNode
{
public:
    GetBinPose(const std::string& name, const BT::NodeConfig& config);
    ~GetBinPose();

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("bin_pose")
        };
    }

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr bin_pose_sub_;
    std::optional<std::shared_ptr<geometry_msgs::msg::PoseStamped>> bin_pose_;
    std::thread spin_thread_;

    void binPoseCallback(geometry_msgs::msg::PoseStamped::UniquePtr msg);
};

} // namespace tennis_demo