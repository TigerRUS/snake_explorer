# Snake Explorer - Autonomous Coverage Path Planner

This ROS1 package implements a coverage path planning algorithm that generates a snake-like pattern to explore a given area while avoiding obstacles. The robot navigates through the generated waypoints and returns to the origin after completing the coverage.

## Features

- Generates efficient snake-like coverage path within specified bounds
- Dynamically adapts to the actual free space in the map
- Handles goal failures and timeouts
- Returns to home position after completing coverage
- Configurable exploration parameters:
  - Coverage resolution (distance between parallel paths)
  - Manual exploration bounds
  - Safety padding around obstacles

## Dependencies

- ROS1
- ROS packages:
  - `roscpp`
  - `std_msgs`
  - `nav_msgs`
  - `geometry_msgs`
  - `tf`
  - `actionlib`
  - `move_base_msgs`
- Navigation stack (for `move_base`)

## Installation

Clone the package to your catkin workspace `src` folder:
```
cd ~/catkin_ws/src
git clone git clone https://github.com/TigerCTPO/qr_detector.git
```

Build the package:
```
cd ~/catkin_ws
catkin_make --only-pkg-with-deps snake_explorer
source devel/setup.bash
```

## Usage

Basic Usage

First, launch your robot's navigation stack and bringup:
```
roslaunch [your_robot_package] bringup.launch
roslaunch [your_navigation_package] navigation.launch
```

Launch the snake explorer:
```
roslaunch snake_explorer snake_explorer.launch
```

## Configuration

The launch file (snake_explorer.launch) includes these key parameters:

- map: Map file name (default: "map")

- nav_use_rotvel: Whether to use rotational velocity for navigation

The snake_explorer node accepts these parameters:

- min_x, max_x, min_y, max_y: bounds for exploration (overrides walls)

- coverage_resolution: Distance between parallel paths (default: 1.0m)

- goal_timeout: Timeout for each goal (default: 20.0s)

- padding: Safety distance from obstacles (default: 1.0m)

## How It Works
Map Processing:

- Receives occupancy grid map from /map topic

- Calculates free space within specified bounds

- Applies safety padding around obstacles

Navigation:

- Sends goals sequentially to move_base

- Handles goal failures by skipping to next waypoint

- Returns to origin (0,0) after completing coverage

- Shuts down node after returning home

## Notes
Requires proper TF tree with map to base_link transform

Depends on a properly configured move_base

For best results, ensure your map is accurate and up-to-date before starting exploration