cmake_minimum_required(VERSION 3.8)
project(tennis_demo_behaviorized)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(spot_behaviors REQUIRED)
find_package(nrg_utility_behaviors REQUIRED)
find_package(nrg_navigation_behaviors REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

include_directories(include)

add_executable(run_tennis_tree
  src/run_tennis_tree.cpp
  src/behaviors/compute_bin_approach_goal.cpp
  src/behaviors/lookup_tf.cpp
  src/behaviors/compute_ball_approach_goal.cpp
  src/behaviors/rotate_pose_yaw.cpp
  src/behaviors/pick_ball.cpp
  src/behaviors/drop_ball.cpp
)

ament_target_dependencies(run_tennis_tree
  rclcpp
  behaviortree_cpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  std_msgs
  spot_behaviors
  nrg_utility_behaviors
  nrg_navigation_behaviors
  ament_index_cpp
  moveit_ros_planning_interface

)

target_link_libraries(run_tennis_tree
  yaml-cpp
)

target_include_directories(run_tennis_tree PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Explicitly set C++20 for the target
set_target_properties(run_tennis_tree PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
)

install(TARGETS run_tennis_tree
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()