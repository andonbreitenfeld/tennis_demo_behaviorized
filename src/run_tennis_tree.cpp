#include <iostream>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/tree_node.h>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Custom behaviors (using YOUR actual include paths)
#include "tennis_behaviorized/behaviors/lookup_tf.hpp"
#include "tennis_behaviorized/behaviors/compute_bin_approach_goal.hpp"

// Utility behaviors
#include <nrg_utility_behaviors/trigger_service.hpp>

// Navigation behaviors
#include <nrg_navigation_behaviors/navigate_to_pose.hpp>

int main(int argc, char** argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    try
    {
        // BehaviorTree factory
        BT::BehaviorTreeFactory factory;

        // --------------------------------------------------------------
        // Register BT nodes used in tennis_tree.xml
        // --------------------------------------------------------------

        // LookupTF node (namespace chair_manipulation)
        factory.registerNodeType<chair_manipulation::LookupTF>("LookupTF");

        // ComputeBinApproachGoal node
        factory.registerNodeType<ComputeBinApproachGoal>("ComputeBinApproachGoal");

        // TriggerService (claim / power_on / undock)
        factory.registerNodeType<nrg_utility_behaviors::TriggerService>("TriggerService");

        // NavigateToPose
        factory.registerNodeType<nrg_navigation_behaviors::NavigateToPose>("NavigateToPose");

        // --------------------------------------------------------------
        // Load the behavior tree XML
        // --------------------------------------------------------------
        std::string share_dir =
            ament_index_cpp::get_package_share_directory("tennis_demo_behaviorized");

        std::string xml_path = share_dir + "/behavior_trees/tennis_tree.xml";
        std::cout << "Loading behavior tree: " << xml_path << std::endl;

        auto tree = factory.createTreeFromFile(xml_path);

        // Run the behavior tree
        tree.tickWhileRunning();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Behavior tree exception: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
