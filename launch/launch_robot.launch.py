# Further details into the ROS2 launch file is here: https://design.ros2.org/articles/roslaunch.html
# For which parameters are also important: http://design.ros2.org/articles/ros_parameters.html
# And node lifecycle: http://design.ros2.org/articles/node_lifecycle.html

# This is our all-in-one launch file.  It will launch:
#   robot_state_publisher
#   joint_state_publisher
#   ros2_controller
#   micro_ros_agent (eventually)

# At this point in the loading process you will need to have joint_state_publisher_gui installed
#   Note that joint_state_publisher also gets installed but we won't use it.  Not directly anyways.
#   sudo apt install ros-humble-joint-state-publisher-gui

# Also possibly joint_state_broadcaster to put the states into /robot_description
#   sudo apt install ros-humble-joint_state_broadcaster



# TODO: Update the launcher to be more event driven.  This will require a bunch more import packages
# See here for more info:  https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Using-Event-Handlers.html


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

# Just a note about colored print statements
#    print ('\033[92m' + package_name, '\033[0m')



def generate_launch_description():
    # pdb.set_trace()  #Python debug break if needed

    # ****** Package Path ****** #
    # First we are going to set a bunch of environment variables.  We are going to set
    # them immediately at the top of the launch file to make it easier to find and
    # change when we build a new project or we want to change something.
    # 
    # package_name: is to be used for string replacement and it needs to be the same as
    # the build directory in.  **See the standard package directory structure for details
    package_name='inmoov_base'

    # Next we use the package_name variable we just set to build out the whole directory
    # get_package_share_directory is a function imported from ament_index_python.packages
    pkg_path = os.path.join(get_package_share_directory(package_name))

    # Now that we have the entire path to our project saved in 'pkg_path' now we
    # can create a path to our core robot description file.  All other URDF files are
    # linked to this core file.  Making this a variable means we can pretty easily
    # change WHICH robot we are launching.
    # TODO: Decide if this needs to be a runtime cmd-line launch argument
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')




    # Convert the XACRO file into straight URDF and store that into a description variable
    # There seems to be two different ways of doing this.
    # I believe the commented line is the old way.  I left it here for reference.
    # robot_description_config = Command(['xacro ', xacro_file])
    robot_description_config = xacro.process_file(xacro_file)


    # ***** Runtime Variables ****** #
    # If we are using gazebo we need to enable 'use_sim_time'.  We may be able to
    # remove this from the launch configuration and configure it in the xacro.
    # use_sim_time = 'true'
    use_sim_time = LaunchConfiguration('use_sim_time')
    rate = LaunchConfiguration('100')
    # print ('\033[92m' + use_sim_time, '\033[0m')



    # ***** Node Parameters ****** 
    # joint_state_publisher_params = {use_sim_time, 'rate': 100}
    # {'robot_description': robot_description_config,}
    joint_state_publisher_params = {'use_sim_time': True, 'rate': 100}
    
    # rViz2 arguements.  They don't seem to work so they are applied manually below in the node definition.
    # I believe they are arguments as oposed to params because it is an executable not a python launch file
    # rviz2_args = {'-d ' + pkg_path + 'config' + 'config_file.rviz'}
    #rviz2_args = {'-d', [os.path.join(pkg_path, 'config', 'config_file.rviz')]}

    

    # ****** Node Definition ****** #
    # Include the robot_state_publisher launch file, provided by our own package. 
    # Force sim time to be enabled
    # Create a launch description object.  
    # It will be referenced at the bottom of this file.
    # the launch_arguments are defined at the entry point xacro file 'robot.urdf.xacro'
    # Rudimentary tutorial on including other launch files is here: https://www.youtube.com/watch?v=sl0exwcg3o8
    # A better tutorial page is here with more advance examples: https://roboticscasual.com/tutorial-ros2-launch-files-all-you-need-to-know/
    # Official documentation for including launch files is here: https://docs.ros.org/en/iron/Tutorials/Intermediate/Launch/Using-Substitutions.html#create-and-setup-the-package
    robot_state_publisher = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    [os.path.join(
                                        get_package_share_directory(package_name),'launch','rsp.launch.py'
                                    )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'false'}.items()
    )   

    # Joint State Publisher.  This provides us with a little GUI where we can change and publish
    # values to the /joint_state topic.  This ultimately isn't the right way to do this.
    # Right now I am moving the sliders which it publishes and microros is reading those values
    # repeatedly and moving the servos.  However I think it is proper for ros2_control to publish
    # these values and then I won't need this later.
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[joint_state_publisher_params]
        # parameters=[use_sim_time]
    )

    # Launching rViz2.  For some reason, I can't set the config arguements above.  
    # TODO: This needs to be an arguement variable or a YAML file.
    rviz2_node = Node(
        package="rviz2",
        # namespace='',
        executable="rviz2",
        name='rviz2',
        arguments=['-d', [os.path.join(get_package_share_directory(package_name), 'config', 'drive_bot.rviz')]]
        # arguments=['-d', [os.path.join(pkg_path, 'config', 'config_file.rviz')]]

    )

    ################# Robot Description ########################
    # In the scope of the current launch file (this may have been launched
    # individually or via rsp.launch.py), we need to load the robot description
    # into a variable again to use for more launch functions.
    # The object below loads teh current robot_description from ROS2
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    # As oposed to this line that loads a brand new variable via the xacro file on disk
    # The potential problem with this is that any on the fly
    # modifications will not be available.
    # I actually think this is giving me a warning when the launcher runs.





    # Launch the ros2_control manager.  Only needed if not using .......
    # First we build a variable and set our control description yaml file.

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    # delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

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





    # params = {'robot_description': 'robot_description'}

    # node_joint_state_publisher = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     output='screen',
    #     parameters=[params]
    # )

    # Launch them all!
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        
        # node_joint_state_publisher,

        # controller_manager,
        # delayed_controller_manager,

        # diff_drive_spawner,
        # delayed_diff_drive_spawner,
        
        # joint_broad_spawner
        # delayed_joint_broad_spawner

        # twist_mux,
        # joystick,
        rviz2_node,
    ])
