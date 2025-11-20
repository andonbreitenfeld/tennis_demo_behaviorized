#include "tennis_demo_behaviorized/behaviors/check_bin_detected.hpp"

namespace tennis_demo
{

CheckBinDetected::CheckBinDetected(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("check_bin_detected_node");
    
    bin_detected_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/bin_detected",
        rclcpp::QoS(10),
        std::bind(&CheckBinDetected::binDetectedCallback, this, std::placeholders::_1)
    );
    
    spin_thread_ = std::thread([this]() {
        rclcpp::spin(node_);
    });
}

CheckBinDetected::~CheckBinDetected()
{
    rclcpp::shutdown();
    if (spin_thread_.joinable())
    {
        spin_thread_.join();
    }
}

BT::NodeStatus CheckBinDetected::tick()
{
    // Wait a little for messages to come through
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    
    if (!bin_detected_.has_value())
    {
        RCLCPP_DEBUG(node_->get_logger(), "No messages received on topic /bin_detected");
        return BT::NodeStatus::FAILURE;
    }
    
    const bool detected = bin_detected_.value();
    RCLCPP_INFO(node_->get_logger(), "Bin is %sdetected", detected ? "" : "not ");
    
    return detected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void CheckBinDetected::binDetectedCallback(std_msgs::msg::Bool::UniquePtr msg)
{
    bin_detected_ = msg->data;
}

} // namespace tennis_demo