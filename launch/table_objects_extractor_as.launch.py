import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('table_plane_extractor')
    config_file = os.path.join(pkg_dir, 'config', 'config.yaml')

    return LaunchDescription([
        Node(
            package='table_plane_extractor',
            executable='table_objects_extractor_as',
            name='table_objects_extractor_as',
            output='screen',
            parameters=[config_file]
        )
    ])