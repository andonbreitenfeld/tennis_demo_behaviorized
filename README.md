# Tennis Demo Behaviorized - Setup Guide

## Package Structure

Your package structure should look like this:

```
tennis_demo_behaviorized/
├── CMakeLists.txt
├── package.xml
├── behavior_trees/
│   └── tennis_tree.xml
├── include/
│   └── tennis_demo_behaviorized/
│       └── behaviors/
│           ├── check_bin_detected.hpp
│           ├── check_ball_detected.hpp
│           ├── get_ball_pose.hpp
│           └── get_bin_pose.hpp
└── src/
    ├── CMakeLists.txt
    ├── run_tennis_tree.cpp
    └── behaviors/
        ├── check_bin_detected.cpp
        ├── check_ball_detected.cpp
        ├── get_ball_pose.cpp
        └── get_bin_pose.cpp
```

## File Placement

1. **Root directory** (`~/project_ws/src/tennis_demo_behaviorized/`):
   - `CMakeLists.txt`
   - `package.xml`

2. **behavior_trees/** directory:
   - `tennis_tree.xml`

3. **include/tennis_demo_behaviorized/behaviors/** directory:
   - `check_bin_detected.hpp`
   - `check_ball_detected.hpp`
   - `get_ball_pose.hpp`
   - `get_bin_pose.hpp`

4. **src/** directory:
   - `run_tennis_tree.cpp`

5. **src/behaviors/** directory:
   - `check_bin_detected.cpp`
   - `check_ball_detected.cpp`
   - `get_ball_pose.cpp`
   - `get_bin_pose.cpp`

## Build Instructions

```bash
cd ~/project_ws
colcon build --packages-select tennis_demo_behaviorized
source install/setup.bash
```

## Running the Demo

```bash
ros2 run tennis_demo_behaviorized run_tennis_tree
```

## Configuration

Before running, update the survey pose coordinates in `src/run_tennis_tree.cpp`:

```cpp
survey_pose->pose.position.x = 2.0;  // Your actual X coordinate
survey_pose->pose.position.y = 1.0;  // Your actual Y coordinate
```

## Topic Interface

The behavior tree expects these topics:

**Subscribed Topics:**
- `/bin_detected` (std_msgs/Bool) - Bin detection status
- `/ball_detected` (std_msgs/Bool) - Ball detection status
- `/ball_base_pose` (geometry_msgs/PoseStamped) - Ball pose for navigation
- `/bin_base_pose` (geometry_msgs/PoseStamped) - Bin pose for navigation

**Required Services:**
- `/spot_driver/power_on` (std_srvs/Trigger) - Power on Spot
- `/spot_driver/undock` (std_srvs/Trigger) - Undock Spot

## Behavior Flow

1. Power on and undock Spot
2. Wait for bin detection (`/bin_detected` = true)
3. Navigate to survey location
4. Wait for ball detection (`/ball_detected` = true)
5. Get ball pose and navigate to it
6. Get bin pose and navigate to it