import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    # Set the path to the observable_pointcloud_map_creator config
    config_path = launch.substitutions.LaunchConfiguration(
        'observable_pointcloud_map_creator_config',
        default=os.path.join(
            get_package_share_directory('observable_pointcloud_map_creator'),
                'config',
                'config_observable_pointcloud_map_creator.yaml'
        )
    )

    observable_pointcloud_map_creator_node = Node(
        package='observable_pointcloud_map_creator',
        executable='observable_pointcloud_map_creator_node',
        name='observable_pointcloud_map_creator_node',
        output='screen',
        parameters=[config_path]
    )
    
    ld.add_action(observable_pointcloud_map_creator_node)

    return ld