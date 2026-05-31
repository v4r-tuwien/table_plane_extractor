import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    package_share_directory = get_package_share_directory('table_plane_extractor')
    
    # Path to the config file
    config_file = os.path.join(package_share_directory, 'config', 'config.yaml')
    
    # Define the node
    table_objects_extractor_server = Node(
        package='table_plane_extractor',
        executable='table_objects_extractor_srv.py',
        name='table_objects_extractor_server',
        output='screen',
        parameters=[config_file]
    )
    
    return LaunchDescription([
        table_objects_extractor_server
    ])