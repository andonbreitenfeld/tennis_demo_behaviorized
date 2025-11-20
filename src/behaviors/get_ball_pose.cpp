#include "tennis_demo_behaviorized/behaviors/get_ball_pose.hpp"

namespace tennis_demo
{

GetBallPose::GetBallPose(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("get_ball_pose_node");
    
    ball_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/ball_base_pose",
        rclcpp::QoS(10),
        std::bind(&GetBallPose::ballPoseCallback, this, std::placeholders::_1)
    );
    
    spin_thread_ = std::thread([this]() {
        rclcpp::spin(node_);
    });
}

GetBallPose::~GetBallPose()
{
    rclcpp::shutdown();
    if (spin_thread_.joinable())
    {
        spin_thread_.join();
    }
}

BT::NodeStatus GetBallPose::tick()
{
    // Wait a little for messages to come through
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    
    if (!ball_pose_.has_value())
    {
        RCLCPP_ERROR(node_->get_logger(), "No messages received on topic /ball_base_pose");
        return BT::NodeStatus::FAILURE;
    }
    
    auto pose = ball_pose_.value();
    setOutput("ball_pose", pose);
    RCLCPP_INFO(node_->get_logger(), "Got ball pose at [%.2f, %.2f, %.2f]", 
                pose->pose.position.x, pose->pose.position.y, pose->pose.position.z);
    
    return BT::NodeStatus::SUCCESS;
}

void GetBallPose::ballPoseCallback(geometry_msgs::msg::PoseStamped::UniquePtr msg)
{
    ball_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(*msg);
}

} // namespace tennis_demo