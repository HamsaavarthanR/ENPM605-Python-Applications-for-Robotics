Terminal1:

colcon build --symlink-install
source install/setup.bash
ros2 launch mapping_navigation_demo navigation_with_map.launch.py

Terminal 2:

source install/setup.bash
ros2 launch cube_nav nav.launch.py