#include <iostream>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/tree_node.h>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Custom behaviors from this package
#include "tennis_demo_behaviorized/behaviors/compute_bin_approach_goal.hpp"
#include "tennis_demo_behaviorized/behaviors/lookup_tf.hpp"

// Utility behaviors
#include <nrg_utility_behaviors/trigger_service.hpp>

// Navigation behavior from nrg_navigation_behaviors
#include <nrg_navigation_behaviors/navigate_to_pose.hpp>

int main(int argc, char** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  try
  {
    // ------------------------------------------------------------------
    // BehaviorTree factory
    // ------------------------------------------------------------------
    BT::BehaviorTreeFactory factory;

    // Register custom nodes
    factory.registerNodeType<chair_manipulation::LookupTF>("LookupTF");
    factory.registerNodeType<ComputeBinApproachGoal>("ComputeBinApproachGoal");

    // TriggerService (claim / power_on / undock)
    factory.registerNodeType<nrg_utility_behaviors::TriggerService>("TriggerService");

    // NavigateToPose from nrg_navigation_behaviors
    factory.registerNodeType<nrg_navigation_behaviors::NavigateToPose>("NavigateToPose");

    // ------------------------------------------------------------------
    // Load behavior tree XML
    // ------------------------------------------------------------------
    const std::string share_dir =
        ament_index_cpp::get_package_share_directory("tennis_demo_behaviorized");
    const std::string xml_path = share_dir + "/behavior_trees/tennis_tree.xml";

    std::cout << "Loading BehaviorTree from: " << xml_path << std::endl;

    auto tree = factory.createTreeFromFile(xml_path);

    // ------------------------------------------------------------------
    // Run the tree until it finishes (SUCCESS or FAILURE)
    // ------------------------------------------------------------------
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
