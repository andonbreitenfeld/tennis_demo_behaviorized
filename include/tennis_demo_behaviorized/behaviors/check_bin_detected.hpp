#pragma once

#include <thread>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/condition_node.h>
#include <std_msgs/msg/bool.hpp>

namespace tennis_demo
{

class CheckBinDetected : public BT::ConditionNode
{
public:
    CheckBinDetected(const std::string& name, const BT::NodeConfig& config);
    ~CheckBinDetected();

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bin_detected_sub_;
    std::optional<bool> bin_detected_;
    std::thread spin_thread_;

    void binDetectedCallback(std_msgs::msg::Bool::UniquePtr msg);
};

} // namespace tennis_demo