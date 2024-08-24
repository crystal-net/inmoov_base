# Author: Paradox
# Date: June 1, 2024
# Description: Display the robotic arm with RViz
 


from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            arguments=['--ros-args', '--log-level', 'info']
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])

