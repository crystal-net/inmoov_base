import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Create launch arguments. As I understand these are locally scoped definitions
    # So we can use them further down to set parameters
    # The text inside the quotes is defined below by the DeclareLaunchArgument of the LauchDescription
    # See more here: https://answers.ros.org/question/322874/ros2-what-is-different-between-declarelaunchargument-and-launchconfiguration/
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('inmoov_base'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    # robot_description_config = Command(['xacro ', xacro_file]) # too simple.
    # sim_mode:= and use_sim_tim is confusing.  I think it should be sim_mode like the parameter to s
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    
    
    # Create a robot_state_publisher node
    # We have to create a new variable object that has the exploded parameter list created by
    # the xacro command and the other parameters above
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time, 'use_ros2_control': use_ros2_control}
    
    
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
       parameters=[params]
    )





    # Build the launch description
    # Each argument has:
    #   - the LaunchConfiguration object defined above.
    #   - a default in case we don't define anything
    #   - a description.  Not sure where this is found.  Maybe on a command line help?     
    return LaunchDescription([
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true'),
    DeclareLaunchArgument(
        'use_ros2_control',
        default_value='false',
        description='Use ros2_control if true'),

        node_robot_state_publisher
    ])
