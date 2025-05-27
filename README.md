# ENPM605 - FINAL PROJECT
ROS2 Navigation Project with ArUco Marker Interaction

![ENPM605-GIF](https://github.com/user-attachments/assets/8aeec311-cc81-4599-9acb-a4a87496b2b0)

## Introduction

The project focuses on implementing an autonomous navigation system using ROS 2 in a
Gazebo simulation environment, where the ‘ros-bot’ robot package must detect and interact with
ArUco-marked cubes while navigating locations predefined in a parameters (.yaml) file based on
marker IDs obtained from the camera. The system integrates mapping, waypoint navigation,
ArUco-marker detection detection, and behavior execution, requiring the robot to dynamically
react to its environment based on detected markers and preconfigured motion parameters.

## Prerequisites

- Clone 'ENPM605_FinalProject_5' repository in your '/src' folder in a new workspace.
- Build the simulation environment.

```bash
colcon build --symlink-install --packages-up-to rosbot --cmake-args -DCMAKE_BUILD_TYPE=Release
```
- Build the Python demo packages:
```bash
colcon build --symlink-install --packages-select ros2_aruco_interfaces ros2_aruco
```

## Testing

Start the environment:

```bash
ros2 launch rosbot_gazebo simulation.launch.py
```

Check the Gazebo camera is reporting the detected AruCo marker:


```bash
ros2 topic echo /aruco_markers
```

## Core Tasks

1. Modifying Husarion World
   - Three Gazebo cylinders (radius: 0.5m, height: 0.5m) were introduced at specified coordinates to challenge navigation and enhance obstacle avoidance logic.
   - Two cubes were placed strategically, each featuring unique ArUco markers for detection and interaction.
   - A Gazebo camera was positioned to monitor Cube #1.
2. Mapping the Environment
  - Use slam_toolbox to create a map of the simulated world, ensuring accurate representation of static obstacles and navigation paths.
  - Store the generated map for later use in navigation tasks.
3. Navigating to Marked Cubes
  - Subscribe to /aruco_markers to detect ArUco marker IDs on cube faces.
  - Convert marker pose from camera frame to map frame using transformation logic.
  - Guide the robot to a position opposite the detected marker and prepare for further movement decisions.
4. Waypoint Navigation Using Parameters
  - Retrieve goal poses from a YAML parameter file, which stores positions, orientations, and navigation behaviors associated with specific marker IDs.
  - Move the robot to different predefined waypoints, ensuring accurate pose execution and parameter-based control.
5. Executing Circling Behavior Around Cube #2
  - Detect the ArUco marker ID on Cube #2 to determine the robot’s final goal.
  - Compute a circular path around the cube using trajectory planning techniques.
  - Execute movement around Cube #2 in a clockwise or counterclockwise direction, depending on parameter settings.
6. Final Navigation Task
  - Use the detected marker ID to retrieve the robot’s final goal location from the parameter file.
  - Navigate to the final destination while considering environmental constraints and obstacle avoidance strategies.


## Run Final Project Package

- Terminal 1:
```bash
colcon build --symlink-install
```
```bash
source install/setup.bash
```
```bash
ros2 launch mapping_navigation_demo navigation_with_map.launch.py
```

- Terminal 2:
```bash
source install/setup.bash
```
```bash
ros2 launch cube_nav nav.launch.py
```

## Team
- Hamsaavarthan Ravichandar
- Manoj Kumar Selvaraj
- David Ajayi

## Links
- Video: https://youtu.be/IVg2Th0wpxA
