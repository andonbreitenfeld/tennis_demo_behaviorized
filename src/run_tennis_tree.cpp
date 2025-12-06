#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/tree_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/buffer.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

// Custom nodes
#include "tennis_demo_behaviorized/behaviors/check_bin_detected.hpp"
#include "tennis_demo_behaviorized/behaviors/check_ball_detected.hpp"
#include "tennis_demo_behaviorized/behaviors/get_ball_pose.hpp"
#include "tennis_demo_behaviorized/behaviors/get_bin_pose.hpp"
#include "tennis_demo_behaviorized/behaviors/pick_ball.hpp"
#include "tennis_demo_behaviorized/behaviors/drop_ball.hpp"

// Utility behaviors
#include <nrg_utility_behaviors/get_current_pose.hpp>
#include <nrg_utility_behaviors/trigger_service.hpp>

// Navigation behaviors
#include <nrg_navigation_behaviors/navigate_to_pose.hpp>

// Spot behaviors
#include <spot_behaviors/walk_to_pose.hpp>

int main(int argc, char **argv)
{
    // Initialize ROS2 client library
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;

    BT::BehaviorTreeFactory factory;
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Register custom nodes
    factory.registerNodeType<tennis_demo::CheckBinDetected>("CheckBinDetected");
    factory.registerNodeType<tennis_demo::CheckBallDetected>("CheckBallDetected");
    factory.registerNodeType<tennis_demo::GetBallPose>("GetBallPose");
    factory.registerNodeType<tennis_demo::GetBinPose>("GetBinPose");
    factory.registerNodeType<tennis_demo::PickBall>("PickBall");
    factory.registerNodeType<DropBall>("DropBall");

    // Register utility nodes
    factory.registerNodeType<nrg_utility_behaviors::TriggerService>("TriggerService");
    factory.registerNodeType<nrg_utility_behaviors::GetCurrentPose>("GetCurrentPose");
    factory.registerNodeType<nrg_navigation_behaviors::NavigateToPose>("NavigateToPose");
    factory.registerNodeType<spot_behaviors::WalkToPose>("WalkToPose", tf_buffer);

    // Load BehaviorTree from installed file
    std::string share_path = ament_index_cpp::get_package_share_directory("tennis_demo_behaviorized");
    std::string xml_path = share_path + "/behavior_trees/tennis_tree.xml";

    // Create the tree
    auto tree = factory.createTreeFromFile(xml_path);

    // Set up survey location pose (adjust coordinates as needed)
    auto survey_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
    survey_pose->header.frame_id = "spot_nav/map";
    survey_pose->pose.position.x = 2.0;  // Replace with actual coordinates (2.0)
    survey_pose->pose.position.y = -4.0;  // Replace with actual coordinates (-4.0)
    survey_pose->pose.position.z = 0.0;
    // Quaternion: w=1, x=0, y=0, z=0 = Roll=0°, Pitch=0°, Yaw=0° (no rotation)
    survey_pose->pose.orientation.x = 0.0;
    survey_pose->pose.orientation.y = 0.0;
    survey_pose->pose.orientation.z = -0.7071;
    survey_pose->pose.orientation.w = 0.7071;

    // Set initial values in blackboard
    tree.rootBlackboard()->set<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("survey_pose", survey_pose);

    // hard code test point for manipulation BT testing
    auto bin_nav_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
    bin_nav_pose->header.frame_id = "spot_nav/map";
    bin_nav_pose->pose.position.x = 1.63907;  // Replace with actual coordinates (2.0)
    bin_nav_pose->pose.position.y = -0.464794;  // Replace with actual coordinates (-4.0)
    bin_nav_pose->pose.position.z = 0.8;
    // Quaternion: w=1, x=0, y=0, z=0 = Roll=0°, Pitch=0°, Yaw=0° (no rotation)
    bin_nav_pose->pose.orientation.x = 0.0;
    bin_nav_pose->pose.orientation.y = 0.0;
    bin_nav_pose->pose.orientation.z = -0.701875;
    bin_nav_pose->pose.orientation.w = 0.7123;
    tree.rootBlackboard()->set<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("bin_nav_pose", bin_nav_pose);

    // Tick the tree
    tree.tickWhileRunning();

    rclcpp::shutdown();
    return 0;
}