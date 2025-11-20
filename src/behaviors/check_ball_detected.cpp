#include "tennis_demo_behaviorized/behaviors/check_ball_detected.hpp"

namespace tennis_demo
{

CheckBallDetected::CheckBallDetected(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("check_ball_detected_node");
    
    ball_detected_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/ball_detected",
        rclcpp::QoS(10),
        std::bind(&CheckBallDetected::ballDetectedCallback, this, std::placeholders::_1)
    );
    
    spin_thread_ = std::thread([this]() {
        rclcpp::spin(node_);
    });
}

CheckBallDetected::~CheckBallDetected()
{
    rclcpp::shutdown();
    if (spin_thread_.joinable())
    {
        spin_thread_.join();
    }
}

BT::NodeStatus CheckBallDetected::tick()
{
    // Wait a little for messages to come through
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    
    if (!ball_detected_.has_value())
    {
        RCLCPP_DEBUG(node_->get_logger(), "No messages received on topic /ball_detected");
        return BT::NodeStatus::FAILURE;
    }
    
    const bool detected = ball_detected_.value();
    RCLCPP_INFO(node_->get_logger(), "Ball is %sdetected", detected ? "" : "not ");
    
    return detected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void CheckBallDetected::ballDetectedCallback(std_msgs::msg::Bool::UniquePtr msg)
{
    ball_detected_ = msg->data;
}

} // namespace tennis_demo