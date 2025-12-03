cmake_minimum_required(VERSION 3.8)
project(tennis_demo_behaviorized)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(spot_behaviors REQUIRED)
find_package(nrg_utility_behaviors REQUIRED)
find_package(nrg_navigation_behaviors REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Specify include directories
include_directories(
  include
)

# Build the executable
add_executable(run_tennis_tree 
  src/run_tennis_tree.cpp
  src/behaviors/compute_bin_approach_goal.cpp
  src/behaviors/lookup_tf.cpp
)

# Link executable with libraries
ament_target_dependencies(run_tennis_tree
  rclcpp
  behaviortree_cpp
  tf2_ros
  geometry_msgs
  std_msgs
  spot_behaviors
  nrg_utility_behaviors
  nrg_navigation_behaviors
  ament_index_cpp
)

# Specify header search path
target_include_directories(run_tennis_tree PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install executable
install(TARGETS 
  run_tennis_tree
  DESTINATION lib/${PROJECT_NAME}
)

# Install behavior trees
install(
  DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()