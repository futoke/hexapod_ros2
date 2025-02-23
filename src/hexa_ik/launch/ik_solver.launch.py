from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hexa_ik',
            executable='ik_solver',
            name='ik_solver',
            output='screen',
        ),
    ])
