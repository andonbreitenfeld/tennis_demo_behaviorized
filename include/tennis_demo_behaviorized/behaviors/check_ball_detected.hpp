#pragma once

#include <thread>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/condition_node.h>
#include <std_msgs/msg/bool.hpp>

namespace tennis_demo
{

class CheckBallDetected : public BT::ConditionNode
{
public:
    CheckBallDetected(const std::string& name, const BT::NodeConfig& config);
    ~CheckBallDetected();

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ball_detected_sub_;
    std::optional<bool> ball_detected_;
    std::thread spin_thread_;

    void ballDetectedCallback(std_msgs::msg::Bool::UniquePtr msg);
};

} // namespace tennis_demo