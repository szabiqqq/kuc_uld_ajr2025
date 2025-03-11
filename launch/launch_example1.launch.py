from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='kuc_uld_ajr2025',
            executable='tervrajz',
            name='tervrajz',
            output='screen'
        ),
    ])