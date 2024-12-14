# Author: Paradox
# Date: June 1, 2024
# Description: Display the robotic arm with RViz
 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
 
def generate_launch_description():
 
    #############################################
    # Define constants for filenames and        #
    # subfolders below the parent package path  #
    #############################################
       
    # Package constants
    package_name = 'inmoov_base'
    pkg_share_description = FindPackageShare(package_name)

    # URDF
    urdf_folder = 'description'
    urdf_filename = 'robot.urdf.xacro'
    default_urdf_model_path = PathJoinSubstitution([pkg_share_description, urdf_folder, urdf_filename])

    # rViz2
    rviz_config_folder = 'config/rviz'
    rviz_config_filename = 'drive_bot.rviz'
    default_rviz_config_path = PathJoinSubstitution([pkg_share_description, rviz_config_folder, rviz_config_filename])
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')


    ################################
    # Set paths to important files #
    ################################
 
    # Launch configuration variables specific to simulation
    # joint_state_publisher_gui is used to manually publish joint states to the /joint_states
    # topic which the robot_state_publisher will transform and send to /tf
    # This tool should not be used as a way to control the robot.  It sends INSTANT states and can
    # cause erratic behavior.  But it is nice for testing and showing what the current state is.
    # If this value is true, a gui will be shown, if false a gui will not be shown but the
    # the joint_state_publisher service will be running proving TF
    jsp_gui = LaunchConfiguration('jsp_gui')
    
    # rViz2 is used to show what the current joint state should look like in relation to the hardware
    
    urdf_model = LaunchConfiguration('urdf_model')
    use_sim_time = LaunchConfiguration('use_sim_time')


    #####################################
    # Declare the launch arguments      #
    # These are arguements that are     #
    # available at the command line     #
    # use the --show-args argument      #
    # to see each options from cmd-line #
    #####################################   
    
    # *Note that there is an in/unless condition on
    # the node launch for this arguement.
    declare_jsp_gui_cmd = DeclareLaunchArgument(
        name='jsp_gui', 
        default_value='true', 
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui')
     
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')
 
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model', 
        default_value=default_urdf_model_path, 
        description='Absolute path to robot urdf file')
 
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RVIZ')
 
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
     


    ####################### 
    # Specify the actions #
    #######################
    
    # Publish the joint state values for the non-fixed joints to /tf file.
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(jsp_gui))
 
    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    start_joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(jsp_gui))
 
    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    robot_description_content = ParameterValue(Command(['xacro ', urdf_model]), value_type=str)
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time, 
            'robot_description': robot_description_content}])
       
    # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{
        'use_sim_time': use_sim_time}])
   
 
 
    ##############################################
    # Create the launch description and populate #
    ##############################################
    ld = LaunchDescription()
 
    # Declare the launch options
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
 
    # Add any actions
    ld.add_action(start_robot_state_publisher_cmd)
    
    ld.add_action(start_joint_state_publisher_cmd)

    ld.add_action(start_joint_state_publisher_gui_cmd)

    ld.add_action(start_rviz_cmd)
 
    return ld