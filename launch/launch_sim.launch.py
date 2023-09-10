import os

from ament_index_python.packages import get_package_share_directory

#LaunchDescription is required for ALL ROS2 Python launch files
from launch import LaunchDescription


from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# The Node function from launch_ros.actions is required 
# for launching nodes (executable packages)
from launch_ros.actions import Node



def generate_launch_description():

    # This file launches all the other launch files.
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='inmoov_base' #<--- CHANGE ME

    # I think this is wrong but I want to figure it out so
    #  that I can create a list of variables.
    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')


    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    # We have to launch Gazebo itself first.  Then the next step below is
    # to launch the spawner to put our robot into the simulation
    launch_gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]), 
                    launch_arguments={'worlds': '/inmoov/ros2_ws/src/inmoov_base/worlds/obstacles.world'}.items()
    )

    # Run the spawner node from the gazebo_ros package to spawn our actual
    # robot into the simulation that is currently running.
    # I think it acutally just forces gazebo to pull the model from the robot_descripton
    # that is loaded in the parameter server.
    # This must be run after gazebo.launch.py
    # The entity name 'my_bot' doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description','-entity', 'my_bot'],
        output='screen'
    )

    # These pertain to our new ros2_controller setup
    # See for more info: https://www.youtube.com/watch?v=4QKsDf1c4hc&t=854s
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"]
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"]
    )

    # Run the the microros agent microros_agent package. 
    launch_microros = Node(
        package='micro_ros_agent', 
        executable='micro_ros_agent',
        arguments=['serial', 
                   '--dev', '/dev/ttyACM0'],
        output='screen'
    )
    
    # Run the the rviz2. 
    launch_rviz2 = Node(
        package='rviz2', 
        executable='rviz2',
        # arguments=["use_sim_time"],
        # arguments=["-d", rviz_config_file],
        output='screen',
        # output='log'
    )

    # Launch teleop_twist_keyboard
    # ros2 run teleop_twist_keyboard teleop_twist_keyboard
    twist_keyboard = Node(
        package='teleop_twist_keyboard', 
        executable='teleop_twist_keyboard',
        output='screen'
    )

    # Launch teleop_twist_joystick
    # ros2 launch teleop_twist_joy teleop-launch.py joy_config:='ps3'
    twist_joystick = Node(
        package='teleop_twist_joy', 
        executable='teleop_twist_joy',
        output='screen'
    )

    Launch_rqt_graph = Node(
        package='rqt_graph',
        executable='rqt_graph',
        output='screen',
    )

    # Launch them all!
    # For testing, I could comment out individual lines
    # Even better would be to create condition variables
    # at the top to determine if they will be launched.
    return LaunchDescription([
        rsp,
        # launch_gazebo,
        launch_rviz2,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        # twist_keyboard,
        Launch_rqt_graph,
        # twist_joystick,
        # launch_microros

    ])
