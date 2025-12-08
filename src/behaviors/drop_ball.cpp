#include "tennis_demo_behaviorized/behaviors/drop_ball.hpp"
#include <future>

DropBall::DropBall(
    const std::string& name, 
    const BT::NodeConfig& config)

    : BT::StatefulActionNode(name, config)

{
    try {
    node_ = rclcpp::Node::make_shared("drop_ball");

    // Get robot_description from move_group's namespace
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
        node_, "/spot_moveit/move_group");
    
    // Wait for the parameter service to be available
    if (!param_client->wait_for_service(std::chrono::seconds(10))) {
        RCLCPP_ERROR(node_->get_logger(), 
            "Parameter service /spot_moveit/move_group not available");
        throw std::runtime_error("move_group parameter service not available");
    }
    
    // Get the robot_description parameter
    auto params = param_client->get_parameters({
        "robot_description",
        "robot_description_semantic",
        });
    if (params.size() < 2) {
        RCLCPP_ERROR(node_->get_logger(), 
            "Could not get required parameters from /spot_moveit/move_group");
        throw std::runtime_error("Required parameters not found");
        }
    
    std::string robot_description = params[0].as_string();
    std::string robot_description_semantic = params[1].as_string();

    // Declare parameters on this node
    node_->declare_parameter("robot_description", robot_description);
    node_->declare_parameter("robot_description_semantic", robot_description_semantic);

    // MoveIt setup
    moveit::planning_interface::MoveGroupInterface::Options move_group_options(
        "arm", "robot_description", "/spot_moveit");
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node_, move_group_options);

    move_group_->setPlanningTime(5.0);
    move_group_->setMaxVelocityScalingFactor(0.2);
    move_group_->setMaxAccelerationScalingFactor(0.2);

    // TF setup
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }
    
    catch (const std::exception& e) {
        std::cerr << "DropBall constructor EXCEPTION: " << e.what() << std::endl;
        throw;  // rethrow so BT can fail clearly
    }
    
}

BT::PortsList DropBall::providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("bin_nav_pose") 
        };
    }


// --- sequential running code ---
BT::NodeStatus DropBall::onStart()
{
    auto pose = getInput<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("bin_nav_pose");
    if (!pose)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "DropBall: missing input port [bin_nav_pose]: %s",
                     pose.error().c_str());
        return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped bin_nav_pose = *pose.value();
    RCLCPP_INFO(node_->get_logger(), "Bin pose (in frame %s) recieved: x=%.3f y=%.3f z=%.3f",
                bin_nav_pose.header.frame_id,
                bin_nav_pose.pose.position.x,
                bin_nav_pose.pose.position.y,
                bin_nav_pose.pose.position.z);



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
    // drop_pose.position.x += 0.6; // values found through trial and error, Spot consistently navigates to left side of tag
    // drop_pose.position.y -= 0.2;
    RCLCPP_INFO(node_->get_logger(),
        "Planning frame: %s",
        move_group_->getPlanningFrame().c_str());

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

    RCLCPP_INFO(node_->get_logger(), "Planning to %s pose: x=%.3f y=%.3f z=%.3f",
                pose_name.c_str(),
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z);

    if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "Plan successful, executing...");
        if (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), "Successfully moved to %s pose", pose_name.c_str());
            // return true;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Execution failed for %s pose", pose_name.c_str());
            // return false;
        }
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Planning failed for %s pose", pose_name.c_str());
    }
}
