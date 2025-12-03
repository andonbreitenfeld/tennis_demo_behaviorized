#include "tennis_demo_behaviorized/behaviors/drop_ball.hpp"

namespace tennis_demo
{

DropBall::DropBall(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("drop_ball");
    
    // MoveIt setup
    moveit::planning_interface::MoveGroupInterface::Options move_group_options(
        "arm", "robot_description", node_->get_namespace());
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node_, move_group_options);

    move_group_->setPlanningTime(5.0);
    move_group_->setMaxVelocityScalingFactor(0.2);
    move_group_->setMaxAccelerationScalingFactor(0.2);

    // TF setup
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Spin ROS in background
    executor_.add_node(node_);
    spin_thread_ = std::thread([this]() { executor_.spin(); });
    
    }
}

DropBall::~DropBall()
{
    // rclcpp::shutdown(); // chat says this could shut down ros on all nodes, should only be called once in main BT executable
    executor_.cancel();
    if (spin_thread_.joinable())
    {
        spin_thread_.join();
    }
}

//prev had 'static' before declaring BT::Ports...
static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("bin_nav_pose")
        };
    }


// --- sequential running code ---
BT::NodeStatus DropBall::onStart()
{
    auto pose = getInput<geometry_msgs::msg::PoseStamped>("bin_nav_pose");
    if (!pose)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "DropBall: missing input port [bin_nav_pose]: %s",
                     pose.error().c_str());
        return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped bin_nav_pose = pose.value();

    // Launch async task
    future_ = std::async(std::launch::async,
                         [this, bin_nav_pose]() {
                             return executePickAndPlace(bin_nav_pose);
                         });

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DropBall::onRunning()
{
    using namespace std::chrono_literals;

    if (future_.wait_for(0ms) == std::future_status::ready)
    {
        bool ok = future_.get();
        return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void DropBall::onHalted()
{
    RCLCPP_WARN(node_->get_logger(), "DropBall halted");
}

// --- transform bin pose and move move to drop pose ---
bool DropBall::executePickAndPlace(const geometry_msgs::msg::PoseStamped& msg)
{
    RCLCPP_INFO(node_->get_logger(), "DropBall: executing motion...");

    // transform input pose (bin pose) to spot body frame first
    geometry_msgs::msg::PoseStamped transformed;
    try {
        auto tf = tf_buffer_->lookupTransform(
            "body",
            msg.header.frame_id,
            rclcpp::Time(0),
            std::chrono::milliseconds(200));

        tf2::doTransform(msg, transformed, tf);

    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(node_->get_logger(), "TF error: %s", ex.what());
        return false;
    }

    transformed.pose = rotatePoseRoll90(transformed.pose);

    // Approach pose
    geometry_msgs::msg::Pose drop_pose = transformed.pose;
    drop_pose.position.x += 0.6; // values found through trial and error, Spot consistently navigates to left side of tag
    drop_pose.position.y -= 0.2;

    moveToPose(drop_pose, "drop");
    rclcpp::sleep_for(std::chrono::seconds(3));

    return true;
}

// --- helper functions ---
geometry_msgs::msg::Pose DropBall::rotatePoseRoll90(const geometry_msgs::msg::Pose &pose_body)
    {
        tf2::Quaternion q_orig;
        tf2::fromMsg(pose_body.orientation, q_orig);

        // Local rotation: 90° about X-axis (roll)
        tf2::Quaternion q_rot;
        q_rot.setRPY(M_PI / 2.0, 0, 0);

        // Local = post-multiply
        tf2::Quaternion q_final = q_orig * q_rot;

        geometry_msgs::msg::Pose out = pose_body;
        out.orientation = tf2::toMsg(q_final);
        return out;
    }

void DropBall::moveToPose(const geometry_msgs::msg::Pose &target_pose, const std::string &pose_name)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group_->setPoseTarget(target_pose);

    RCLCPP_INFO(LOGGER, "Planning to %s pose: x=%.3f y=%.3f z=%.3f",
                pose_name.c_str(),
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z);

    if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(LOGGER, "Plan successful, executing...");
        if (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(LOGGER, "Successfully moved to %s pose", pose_name.c_str());
            // return true;
        } else {
            RCLCPP_ERROR(LOGGER, "Execution failed for %s pose", pose_name.c_str());
            // return false;
        }
    } else {
        RCLCPP_ERROR(LOGGER, "Planning failed for %s pose", pose_name.c_str());
    }
}

 // namespace tennis_demo