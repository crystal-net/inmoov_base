import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
# from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare

import xacro

# The joint_state_publisher WILL cause conflicts when running along side the joint controllers
# that are defined in the config/controllers.yaml files
# It is possible to run them side by side but there will be artifacts and glitches.
# joint_state_publisher is ONLY for testing the URDF state and composistion in rViz2


def generate_launch_description():
    
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('inmoov_base'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    
    # Create a robot_state_publisher node
    # params = {'robot_description': xacro_file,}
    # This configuration below is just a more updated avanced way of doing it.
    # I wanted this here as a reminder of what else is possible

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("inmoov_base"),
            "config",
            "gazebo_controllers.yaml",
        ]
    )

    # use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    # Get URDF via xacro
    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution(
    #             [FindPackageShare("inmoov_base"), "description", "robot.urdf.xacro"]
    #         ),
    #         " ",
    #         "use_mock_hardware:=",
    #         use_mock_hardware,
    #     ]
    # )

    robot_description = {"robot_description": xacro_file,}


    node_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )



    # Launch!
    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'use_sim_time',
        #     default_value='false',
        #     description='Use sim time if true'),
        # DeclareLaunchArgument(
        #     'use_ros2_control',
        #     default_value='true',
        #     description='Use ros2_control if true'),

        node_control_node
    ])

# Next, launch gazebo.launch.py