# Author: Paradox
# Date: June 1, 2024
# Description: Display the robotic arm with RViz
 


from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=["serial", "--dev", "/dev/ttyACM0"]
            # arguments=["udp4", "-p", "8888", "-v6"]
        )
    ])

