import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Build an object of type LaunchConfiguration.  I don't know how this is supposed to work.
    use_sim_time = LaunchConfiguration('use_sim_time')
    # use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Now lets build the path to our URDF description.  These are split up so that one portion of the
    # the path can be changed or defined without modifying the rest.

    # First we get the root path of the project directory
    # pkg_path = os.path.join(get_package_share_directory('articubot_one'))
    pkg_path = os.path.join(get_package_share_directory('inmoov_base'))
    
    # Next we append the sub-directory path to the URDF.XACRO itself
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')

    # Lastly we build a raw URDF out of the xacro file and store the contents
    # It is not a URDF file per se but the contents had we actually created one.
    robot_description_config = Command(['xacro ', xacro_file])


    # This is other setup information that I don't think we need as we are using DeclareLaunchArguement below
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    # robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    # robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control])
    
    
    # And finally we build the list of parameters for launching, 
    # use_sim_time is a little confusion here as it is the same name for two different contexts
    # The use_sim_time on the left is the ros2 parameter vs the right which holds a variable of true or false 
    # params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time, 'rate': "100"}
    
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),

        node_robot_state_publisher
    ])


# Next, launch jsp.launch.py