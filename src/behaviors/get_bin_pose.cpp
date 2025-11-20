#include "tennis_demo_behaviorized/behaviors/get_bin_pose.hpp"

namespace tennis_demo
{

GetBinPose::GetBinPose(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("get_bin_pose_node");
    
    bin_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/bin_base_pose",
        rclcpp::QoS(10),
        std::bind(&GetBinPose::binPoseCallback, this, std::placeholders::_1)
    );
    
    spin_thread_ = std::thread([this]() {
        rclcpp::spin(node_);
    });
}

GetBinPose::~GetBinPose()
{
    rclcpp::shutdown();
    if (spin_thread_.joinable())
    {
        spin_thread_.join();
    }
}

BT::NodeStatus GetBinPose::tick()
{
    // Wait a little for messages to come through
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    
    if (!bin_pose_.has_value())
    {
        RCLCPP_ERROR(node_->get_logger(), "No messages received on topic /bin_base_pose");
        return BT::NodeStatus::FAILURE;
    }
    
    auto pose = bin_pose_.value();
    setOutput("bin_pose", pose);
    RCLCPP_INFO(node_->get_logger(), "Got bin pose at [%.2f, %.2f, %.2f]", 
                pose->pose.position.x, pose->pose.position.y, pose->pose.position.z);
    
    return BT::NodeStatus::SUCCESS;
}

void GetBinPose::binPoseCallback(geometry_msgs::msg::PoseStamped::UniquePtr msg)
{
    bin_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(*msg);
}

} // namespace tennis_demo