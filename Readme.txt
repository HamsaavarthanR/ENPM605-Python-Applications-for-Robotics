# ENPM605 - FINAL PROJECT
Package for the final project

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


## Run Final Project Package

- Terminal1:
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

