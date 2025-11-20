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


   // Set up survey location pose A
   auto survey_pose_a = std::make_shared<geometry_msgs::msg::PoseStamped>();
   survey_pose_a->header.frame_id = "spot_nav/map";
   survey_pose_a->pose.position.x = 3.0;
   survey_pose_a->pose.position.y = -5.0;
   survey_pose_a->pose.position.z = 0.0;
   survey_pose_a->pose.orientation.x = 0.0;
   survey_pose_a->pose.orientation.y = 0.0;
   survey_pose_a->pose.orientation.z = 0.0;
   survey_pose_a->pose.orientation.w = 1.0;


   // Set up survey location pose B
   auto survey_pose_b = std::make_shared<geometry_msgs::msg::PoseStamped>();
   survey_pose_b->header.frame_id = "spot_nav/map";
   survey_pose_b->pose.position.x = 2.0;
   survey_pose_b->pose.position.y = -4.0;
   survey_pose_b->pose.position.z = 0.0;
   survey_pose_b->pose.orientation.x = 0.0;
   survey_pose_b->pose.orientation.y = 0.0;
   survey_pose_b->pose.orientation.z = 0.0;
   survey_pose_b->pose.orientation.w = 1.0;


   // NO @ prefix - just like your working code
   tree.rootBlackboard()->set<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("survey_pose_a", survey_pose_a);
   tree.rootBlackboard()->set<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("survey_pose_b", survey_pose_b);


   // Tick the tree
   tree.tickWhileRunning();


   rclcpp::shutdown();
   return 0;
}
