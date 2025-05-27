from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('cube_nav')
    param_file = os.path.join(pkg_share, 'config', 'cube_nav_params.yaml')
    print(param_file)

    return LaunchDescription([
        Node(
            package='cube_nav',
            executable='nav',
            name='cube_navigator_node',
            parameters=[param_file],
        )
    ])