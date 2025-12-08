#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include "tennis_demo_behaviorized/behaviors/compute_bin_approach_goal.hpp"
#include "tennis_demo_behaviorized/behaviors/compute_ball_approach_goal.hpp"
#include "tennis_demo_behaviorized/behaviors/lookup_tf.hpp"
#include "tennis_demo_behaviorized/behaviors/rotate_pose_yaw.hpp"

#include <nrg_utility_behaviors/trigger_service.hpp>
#include <nrg_utility_behaviors/get_message_from_topic.hpp>
#include <nrg_utility_behaviors/get_current_pose.hpp>
#include <nrg_navigation_behaviors/navigate_to_pose.hpp>
#include <spot_behaviors/walk_to_pose.hpp>

namespace
{

void registerBehaviors(BT::BehaviorTreeFactory& factory,
                       const rclcpp::Node::SharedPtr& node,
                       const std::shared_ptr<tf2_ros::Buffer>& tf_buffer)
{
  factory.registerNodeType<chair_manipulation::LookupTF>("LookupTF");
  factory.registerNodeType<tennis_demo_behaviorized::RotatePoseYaw>("RotatePoseYaw");

  factory.registerBuilder<tennis_demo_behaviorized::ComputeBinApproachGoal>(
    "ComputeBinApproachGoal",
    [node](const std::string& name, const BT::NodeConfig& config) {
      return std::make_unique<tennis_demo_behaviorized::ComputeBinApproachGoal>(
        name, config, node);
    });

  factory.registerBuilder<tennis_demo_behaviorized::ComputeBallApproachGoal>(
    "ComputeBallApproachGoal",
    [node, tf_buffer](const std::string& name, const BT::NodeConfig& config) {
      return std::make_unique<tennis_demo_behaviorized::ComputeBallApproachGoal>(
        name, config, node, tf_buffer);
    });

  factory.registerNodeType<nrg_utility_behaviors::TriggerService>("TriggerService");
  factory.registerNodeType<nrg_utility_behaviors::GetMessageFromTopic>("GetMessageFromTopic");
  factory.registerNodeType<nrg_navigation_behaviors::NavigateToPose>("NavigateToPose");

  factory.registerBuilder<nrg_utility_behaviors::GetCurrentPose>(
    "GetCurrentPose",
    [tf_buffer](const std::string& name, const BT::NodeConfig& config) {
      return std::make_unique<nrg_utility_behaviors::GetCurrentPose>(
        name, config, std::weak_ptr<rclcpp::Node>{}, "", tf_buffer);
    });

  factory.registerBuilder<spot_behaviors::WalkToPose>(
    "WalkToPose",
    [tf_buffer](const std::string& name, const BT::NodeConfig& config) {
      return std::make_unique<spot_behaviors::WalkToPose>(name, config, tf_buffer);
    });
}

geometry_msgs::msg::PoseStamped::SharedPtr parsePoseFromYaml(const YAML::Node& wp)
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
}

void loadWaypoints(BT::Blackboard::Ptr blackboard, const std::string& share_dir)
{
  const std::string yaml_path = share_dir + "/config/waypoints.yaml";
  YAML::Node config = YAML::LoadFile(yaml_path);
  
  if (!config["waypoints"] || config["waypoints"].size() < 3) {
    throw std::runtime_error("waypoints.yaml must contain at least 3 waypoints");
  }

  blackboard->set("poseA", parsePoseFromYaml(config["waypoints"][0]));
  blackboard->set("poseB", parsePoseFromYaml(config["waypoints"][1]));
  blackboard->set("poseC", parsePoseFromYaml(config["waypoints"][2]));
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = rclcpp::Node::make_shared("tennis_demo_behaviorized_bt");
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    BT::BehaviorTreeFactory factory;
    registerBehaviors(factory, node, tf_buffer);

    const std::string share_dir = 
      ament_index_cpp::get_package_share_directory("tennis_demo_behaviorized");
    const std::string xml_path = share_dir + "/behavior_trees/tennis_tree.xml";

    auto tree = factory.createTreeFromFile(xml_path);
    loadWaypoints(tree.rootBlackboard(), share_dir);

    tree.tickWhileRunning();
  }
  catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}