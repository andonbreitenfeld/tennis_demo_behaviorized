#include "tennis_demo_behaviorized/behaviors/pick_ball.hpp"
#include <future>

PickBall::PickBall(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config)
{
    try {

    node_ = rclcpp::Node::make_shared("pick_ball");
    
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
    
    // Check robot_description
    if (params[0].get_type() == rclcpp::PARAMETER_NOT_SET) {
        RCLCPP_ERROR(node_->get_logger(), "robot_description not found");
        throw std::runtime_error("robot_description not found");
    }
    
    // Check robot_description_semantic
    if (params[1].get_type() == rclcpp::PARAMETER_NOT_SET) {
        RCLCPP_ERROR(node_->get_logger(), "robot_description_semantic not found");
        throw std::runtime_error("robot_description_semantic not found");
    }

    std::string robot_description = params[0].as_string();
    std::string robot_description_semantic = params[1].as_string();

    // Declare parameters on this node
    node_->declare_parameter("robot_description", robot_description);
    node_->declare_parameter("robot_description_semantic", robot_description_semantic);

    // MoveIt setup
    moveit::planning_interface::MoveGroupInterface::Options move_group_options(
        "arm", "robot_description", "/spot_moveit/robot_description");
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
    
    catch (const std::exception& e) {
        std::cerr << "PickBall constructor EXCEPTION: " << e.what() << std::endl;
        throw;  // rethrow so BT can fail clearly
    }
}

PickBall::~PickBall()
{
    // rclcpp::shutdown(); // chat says this could shut down ros on all nodes, should only be called once in main BT executable
    executor_.cancel();
    if (spin_thread_.joinable())
    {
        spin_thread_.join();
    }
}

//prev had 'static' before declaring BT::Ports...
BT::PortsList PickBall::providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("ball_pose")
        };
    }


// --- sequential running code ---
BT::NodeStatus PickBall::onStart()
{
    auto pose = getInput<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("ball_pose");
    if (!pose)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "PickBall: missing input port [ball_pose]: %s",
                     pose.error().c_str());
        return BT::NodeStatus::FAILURE;
    }

    // Dereferencing shared pointer, keeps Andon's shared_ptr structure in place
    geometry_msgs::msg::PoseStamped ball_pose = *pose.value();

    // Launch async task
    future_ = std::async(std::launch::async,
                         [this, ball_pose]() {
                             return executePickAndPlace(ball_pose);
                         });

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PickBall::onRunning()
{
    using namespace std::chrono_literals;

    if (future_.wait_for(0ms) == std::future_status::ready)
    {
        bool ok = future_.get();
        return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void PickBall::onHalted()
{
    RCLCPP_WARN(node_->get_logger(), "PickBall halted");
}

// --- move to approach and target pose logic ---
bool PickBall::executePickAndPlace(const geometry_msgs::msg::PoseStamped& msg)
{
    RCLCPP_INFO(node_->get_logger(), "PickBall: executing motion...");

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
    geometry_msgs::msg::Pose approach = transformed.pose;
    approach.position.x -= 0.05;
    approach.position.y += 0.03;

    moveToPose(approach, "approach");
    rclcpp::sleep_for(std::chrono::seconds(3));

    // Grasp pose with offsets
    geometry_msgs::msg::Pose grasp = transformed.pose;
    grasp.position.x += 0.03;
    grasp.position.y += 0.03;

    moveToPose(grasp, "grasp");

    return true;
}

// --- helper functions ---
geometry_msgs::msg::Pose PickBall::rotatePoseRoll90(const geometry_msgs::msg::Pose &pose_body)
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

void PickBall::moveToPose(const geometry_msgs::msg::Pose &target_pose, const std::string &pose_name)
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

