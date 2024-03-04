# This is our all-in-one launch file.  It will launch:
#   robot_state_publisher
#   joint_state_publisher
#   ros2_controller
#   micro_ros_agent (eventually)

import pdb
import os
import xacro


from launch import LaunchDescription  # Generat_launch_description() functions
from launch.actions import IncludeLaunchDescription # Include other launch files allowing seperation
from launch.actions import DeclareLaunchArgument  # For launch parameters


from launch.launch_description_sources import PythonLaunchDescriptionSource  # Required for including other python launch files
# from launch_xml.launch_description_sources import XMLLaunchDescriptionSource  # Required for including other XML launch files
# from launch_yaml.launch_description_sources import YAMLLaunchDescriptionSource  # Required for including other YAML launch files

from ament_index_python.packages import get_package_share_directory  #Required for including other pythong launch files
from launch.substitutions import LaunchConfiguration  #String substitution for Python Launch files
from launch.substitutions import Command  #String substitution for inline xacro translation to translate from xacro to urdf

from launch.actions import RegisterEventHandler  #For launching delayed processes
from launch.event_handlers import OnProcessStart  #For launching delayed processes
from launch.actions import TimerAction  #For launching delayed processes
from launch_ros.actions import Node  # Allows for launching cpp nodes from python launch files
# from launch_ros.substitutions import  #String substitution for ROS to find our URDF


def generate_launch_description():
    # pdb.set_trace()  #Python debug break if needed
   
    # We set a package_name to be used for string replacement later in the launch file.
    # It would be advisable to ensure that this is the same as the build directory.
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
    # See the standard package directory structure for details
    package_name='inmoov_base'
    print ('\033[92m' + package_name, '\033[0m')

    pkg_path = os.path.join(get_package_share_directory('inmoov_base'))
    print ('\033[92m' + pkg_path, '\033[0m')

    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    print ('\033[92m' + xacro_file, '\033[0m')

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    # print ('\033[92m' + robot_description, '\033[0m')

    robot_description_config = Command(['xacro ', xacro_file])
    # print ('\033[92m' + robot_description_config, '\033[0m')

    # use_sim_time = 'true'
    use_sim_time = LaunchConfiguration('use_sim_time')
    # print ('\033[92m' + use_sim_time, '\033[0m')


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
    # Rudimentary tutorial on including other launch files is here: https://www.youtube.com/watch?v=sl0exwcg3o8
    # A better tutorial page is here with more advance examples: https://roboticscasual.com/tutorial-ros2-launch-files-all-you-need-to-know/
    # Official documentation for including launch files is here: https://docs.ros.org/en/iron/Tutorials/Intermediate/Launch/Using-Substitutions.html#create-and-setup-the-package
    print ('\033[92m' + "Starting robot_state_publisher", '\033[0m')
    robot_state_publisher = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    [os.path.join(
                                        get_package_share_directory(package_name),'launch','rsp.launch.py'
                                    )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'false'}.items()
    )   

    # I think this list is broken because why do I need a robot description.
    # It seems to load it just fine without specifying it here.
    # params = {'robot_description': robot_description_config,
    #           'use_sim_time': use_sim_time,
    #           'rate': "100"}

    joint_state_publisher_params = {'use_sim_time': True,
                                    'rate': 100}

    print ('\033[92m' + "Starting node_joint_state_publisher_gui", '\033[0m')
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[joint_state_publisher_params]
        # parameters=[{'rate': 100,'use_sim_time': True}]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[],
    )




    # Launch the ros2_control manager.  Only needed if not using ros.
    # First we build a variable and set our control description yaml file.

    print ('\033[92m' + "Starting ros2_control Controller Manager node", '\033[0m')
    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file]
    )




    # delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    # diff_drive_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner.py",
    #     arguments=["diff_cont"],
    # )

    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=controller_manager,
    #         on_start=[diff_drive_spawner],
    #     )
    # )

    # joint_broad_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner.py",
    #     arguments=["joint_broad"],
    # )

    # delayed_joint_broad_spawner = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=controller_manager,
    #         on_start=[joint_broad_spawner],
    #     )
    # )


    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )])
    )


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
        controller_manager,
        joystick,
        rviz_node,
        # diff_drive_spawner,
        # twist_mux,
        # delayed_controller_manager,
        # delayed_diff_drive_spawner,
        # delayed_joint_broad_spawner
    ])
