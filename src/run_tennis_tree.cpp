#include <iostream>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/tree_node.h>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Custom behaviors from this package
#include "tennis_demo_behaviorized/behaviors/compute_bin_approach_goal.hpp"
#include "tennis_demo_behaviorized/behaviors/compute_ball_approach_goal.hpp"
#include "tennis_demo_behaviorized/behaviors/lookup_tf.hpp"

// NRG behaviors
#include <nrg_utility_behaviors/trigger_service.hpp>
#include <nrg_utility_behaviors/get_message_from_topic.hpp>

// Spot behaviors
#include <spot_behaviors/walk_to_pose.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  try
  {
    // Shared TF buffer for WalkToPose
    auto tf_buffer   = std::make_shared<tf2_ros::Buffer>(rclcpp::Clock::make_shared());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    BT::BehaviorTreeFactory factory;

    // Custom nodes from this package
    factory.registerNodeType<chair_manipulation::LookupTF>("LookupTF");
    factory.registerNodeType<ComputeBinApproachGoal>("ComputeBinApproachGoal");
    factory.registerNodeType<tennis_demo_behaviorized::ComputeBallApproachGoal>("ComputeBallApproachGoal");

    // NRG utility behaviors
    factory.registerNodeType<nrg_utility_behaviors::TriggerService>("TriggerService");
    factory.registerNodeType<nrg_utility_behaviors::GetMessageFromTopic>("GetMessageFromTopic");

    // WalkToPose from spot_behaviors
    factory.registerBuilder<spot_behaviors::WalkToPose>(
      "WalkToPose",
      [tf_buffer](const std::string& name, const BT::NodeConfig& config)
      {
        return std::make_unique<spot_behaviors::WalkToPose>(name, config, tf_buffer);
      });

    // Locate BT XML
    const std::string share_dir =
        ament_index_cpp::get_package_share_directory("tennis_demo_behaviorized");
    const std::string xml_path = share_dir + "/behavior_trees/tennis_tree.xml";

    std::cout << "Loading BehaviorTree from: " << xml_path << std::endl;
    auto tree = factory.createTreeFromFile(xml_path);

    // Load waypoints from YAML
    const std::string yaml_path = share_dir + "/config/waypoints.yaml";
    std::cout << "Loading waypoints from: " << yaml_path << std::endl;

    YAML::Node config = YAML::LoadFile(yaml_path);
    if (!config["waypoints"] || config["waypoints"].size() < 2)
    {
      throw std::runtime_error("waypoints.yaml must contain at least 2 waypoints");
    }

    auto make_pose_from_yaml = [](const YAML::Node& wp)
      -> geometry_msgs::msg::PoseStamped::SharedPtr
    {
      auto pose = std::make_shared<geometry_msgs::msg::PoseStamped>();

      pose->header.frame_id = wp["frame_id"].as<std::string>();

      pose->pose.position.x = wp["position"]["x"].as<double>();
      pose->pose.position.y = wp["position"]["y"].as<double>();
      pose->pose.position.z = wp["position"]["z"].as<double>();

      pose->pose.orientation.x = wp["orientation"]["x"].as<double>();
      pose->pose.orientation.y = wp["orientation"]["y"].as<double>();
      pose->pose.orientation.z = wp["orientation"]["z"].as<double>();
      pose->pose.orientation.w = wp["orientation"]["w"].as<double>();

      return pose;
    };

    const auto& wp0 = config["waypoints"][0];
    const auto& wp1 = config["waypoints"][1];

    auto poseA = make_pose_from_yaml(wp0);
    auto poseB = make_pose_from_yaml(wp1);

    // Put them on the root blackboard
    auto bb = tree.rootBlackboard();
    bb->set("poseA", poseA);
    bb->set("poseB", poseB);

    // Run the tree
    tree.tickWhileRunning();
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception while running behavior tree: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
