# This is our all-in-one launch file.  It will launch:
#   robot_state_publisher
#   joint_state_publisher
#   ros2_controller
#   micro_ros_agent (eventually)


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node
import xacro

def generate_launch_description():




   
    # We set a package_name to be used for string replacement later in the launch file.
    # It would be advisable to ensure that this is the same as the build directory.
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
    # See the standard package directory structure for details
    package_name='inmoov_base'
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_path = os.path.join(get_package_share_directory('inmoov_base'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    robot_description_config = Command(['xacro ', xacro_file])

    # Process the URDF file
    # pkg_path = os.path.join(get_package_share_directory(package_name))
    # xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file)

    print ('\033[92m' + "loading", package_name, '\033[0m')

    # Include the robot_state_publisher launch file, provided by our own package. 
    # Force sim time to be enabled
    # Create a launch description object.  
    # It will be referenced at the bottom of this file.
    # the launch_arguments are defined at the entry point xacro file 'robot.urdf.xacro'
    print ('\033[92m' + "Starting robot_state_publisher", '\033[0m')
    robot_state_publisher = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )


    params = {'robot_description': robot_description_config,
              'use_sim_time': use_sim_time,
              'rate': "100"}

    print ('\033[92m' + "Starting node_joint_state_publisher_gui", '\033[0m')
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[params]
    )


    # Launch the ros2_control manager.  Only needed if not using ros.
    # First we build a variable and set our control description yaml file.

    print ('\033[92m' + "Starting ros2_control Controller Manager node", '\033[0m')
    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])
s
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )


    # joystick = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(package_name),'launch','joystick.launch.py'
    #             )])
    # )


    # twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    # twist_mux = Node(
    #         package="twist_mux",
    #         executable="twist_mux",
    #         parameters=[twist_mux_params],
    #         remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    #     )



    # Code for delaying a node (I haven't tested how effective it is)
    # 
    # First add the below lines to imports
    # from launch.actions import RegisterEventHandler
    # from launch.event_handlers import OnProcessExit
    #
    # Then add the following below the current diff_drive_spawner
    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[diff_drive_spawner],
    #     )
    # )
    #
    # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner



    # Launch them all!
    return LaunchDescription([
        robot_state_publisher,
        node_joint_state_publisher_gui,
        # joystick,
        # twist_mux,
        # delayed_controller_manager,
        # delayed_diff_drive_spawner,
        # delayed_joint_broad_spawner
    ])
