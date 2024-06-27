# Author: Addison Sears-Collins
# Date: April 14, 2024
# Description: Launch a robotic arm in Gazebo 
import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import xacro





  ####################################################
  # Configuration Definitions                        #
  # Make changes to startup here                     #
  #                                                  #
  ####################################################

def generate_launch_description():


  # Define constants for filenames and subfolders below the parent package path    
  package_name = 'inmoov_base'
  default_robot_name = 'inmoov'


  urdf_folder = 'description'
  urdf_filename = 'robot.urdf.xacro'

  rviz_package_name = 'ros_rviz2'  #Dont' know if this is even right.  Not currently using it anyways
  rviz_config_folder = 'config/rviz'
  rviz_config_filename = 'drive_bot.rviz'  #Example: '/mycobot_280_arduino_view_description.rviz'

  # package_name_gazebo = 'inmoov_base'
  gazebo_package_name = 'ros_gz_sim' #This allows changing from classic, ignition, or gz
  gazebo_launch_file_path = 'launch'
  gazebo_models_folder = 'models'
  gazebo_config_file = 'config/gz/ros_gz_bridge.yaml'
  gazebo_config_folder = 'config/gz'
  gazebo_world_file = 'empty.world'    # e.g. 'world/empty.world', 'world/house.world'
  gazebo_world_folder = 'worlds'
  

  # Build paths to important files using the variables set above
  pkg_share_description = FindPackageShare(package_name)
  default_urdf_path = PathJoinSubstitution([pkg_share_description, urdf_folder, urdf_filename])

  pkg_ros_rviz = FindPackageShare(rviz_package_name)
  default_rviz_config_path = PathJoinSubstitution([pkg_share_description, rviz_config_folder, rviz_config_filename])

  # pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')  #Depricated?
  
  # pkg_ros_gz_sim = FindPackageShare(gazebo_package_name)  
  pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')  
  default_ros_gz_bridge_config_file_path = PathJoinSubstitution([pkg_share_description, gazebo_config_file])
  default_gazebo_launch_file_path = PathJoinSubstitution([pkg_share_description, gazebo_launch_file_path])
  default_gazebo_models_path = PathJoinSubstitution([pkg_share_description, gazebo_models_folder])
  default_gazebo_world_path = PathJoinSubstitution([pkg_share_description, gazebo_world_folder, gazebo_world_file])

  ros_gz_bridge_config_file_path = PathJoinSubstitution([gazebo_config_folder, gazebo_config_file])


  # Instantiate DeclarLaunchArgument object variables.  Declaration done 
  headless = LaunchConfiguration('headless')
  robot_name = LaunchConfiguration('robot_name')
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  urdf_model = LaunchConfiguration('urdf_model')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_rviz = LaunchConfiguration('use_rviz')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')
  

  # Set the default pose
  x = LaunchConfiguration('x')
  y = LaunchConfiguration('y')
  z = LaunchConfiguration('z')
  roll = LaunchConfiguration('roll')
  pitch = LaunchConfiguration('pitch')
  yaw = LaunchConfiguration('yaw')
  

  # Declare the launch arguments  
  declare_robot_name_cmd = DeclareLaunchArgument(
    name='robot_name',
    default_value=default_robot_name,
    description='The name for the robot')

  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')

  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Display the Gazebo GUI if False, otherwise run in headless mode')

  declare_urdf_model_path_cmd = DeclareLaunchArgument(
    name='urdf_model', 
    default_value=default_urdf_path, 
    description='Absolute path to robot urdf file')
    
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument( 
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')

  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')
    
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')

  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start Gazebo')

  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=default_gazebo_world_path,
    description='Full path to the world model file to load')

  declare_x_cmd = DeclareLaunchArgument(
    name='x',
    default_value='0.0',
    description='x component of initial position, meters')

  declare_y_cmd = DeclareLaunchArgument(
    name='y',
    default_value='0.0',
    description='y component of initial position, meters')
    
  declare_z_cmd = DeclareLaunchArgument(
    name='z',
    default_value='0.05',
    description='z component of initial position, meters')
    
  declare_roll_cmd = DeclareLaunchArgument(
    name='roll',
    default_value='0.0',
    description='roll angle of initial orientation, radians')

  declare_pitch_cmd = DeclareLaunchArgument(
    name='pitch',
    default_value='0.0',
    description='pitch angle of initial orientation, radians')

  declare_yaw_cmd = DeclareLaunchArgument(
    name='yaw',
    default_value='0.0',
    description='yaw angle of initial orientation, radians')


  # Specify the actions

  set_env_vars_resources = AppendEnvironmentVariable(
    'GZ_SIM_RESOURCE_PATH',
    default_gazebo_models_path)








  ####################################################
  # End config file changes                          #
  # Start up functions follow                        #
  # Do not make any changes below this line          #
  ####################################################

  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items())

  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    launch_arguments={'gz_args': '-g -v4 '}.items(),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
    
  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  robot_description_content = ParameterValue(Command(['xacro ', default_urdf_path]), value_type=str)
  # robot_description = xacro.parse(open(default_urdf_path)).to
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{
      'use_sim_time': use_sim_time, 
      'robot_description': robot_description_content}])

  # # Launch RViz
  # start_rviz_cmd = Node(
  #   condition=IfCondition(use_rviz),
  #   package='rviz2',
  #   executable='rviz2',
  #   name='rviz2',
  #   output='screen',
  #   arguments=['-d', rviz_config_file])  
    
  # Spawn the robot
  start_gazebo_ros_spawner_cmd = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
      '-name', robot_name,
      '-topic', "robot_description"
      '-x', x,
      '-y', y,
      '-z', z,
      '-R', roll,
      '-P', pitch,
      '-Y', yaw
      ],
    output='screen')  
    
  # # Bridge ROS topics and Gazebo messages for establishing communication
  start_gazebo_ros_bridge_cmd = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    parameters=[{
      'config_file': default_ros_gz_bridge_config_file_path,
    }],
    output='screen'
  )  
    
  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_robot_name_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_urdf_model_path_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_world_cmd)

  ld.add_action(declare_x_cmd)
  ld.add_action(declare_y_cmd)
  ld.add_action(declare_z_cmd)
  ld.add_action(declare_roll_cmd)
  ld.add_action(declare_pitch_cmd)
  ld.add_action(declare_yaw_cmd)  

  # Add any actions
  ld.add_action(set_env_vars_resources)
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
  ld.add_action(start_robot_state_publisher_cmd)
  # ld.add_action(start_rviz_cmd)
  
  ld.add_action(start_gazebo_ros_spawner_cmd)
  ld.add_action(start_gazebo_ros_bridge_cmd)

  return ld



