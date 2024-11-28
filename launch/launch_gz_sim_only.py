import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


# From the old file.  Probably don't need these any more
# from launch.conditions import IfCondition
# from launch.substitutions import Command, LaunchConfiguration, PythonExpression, PathJoinSubstitution
# from launch_ros.parameter_descriptions import ParameterValue
# from launch_ros.substitutions import FindPackageShare






  ####################################################
  # Configuration Definitions                        #
  # Make changes to startup here                     #
  #                                                  #
  ####################################################

def generate_launch_description():


  # Define constants for filenames and subfolders below the parent package path    
  package_name = 'inmoov_base'
  robot_name = 'inmoov'
  urdf_path = 'description/robot.urdf.xacro'

  pathModelFile = os.path.join(get_package_share_directory(package_name),urdf_path)
  robotDescription = xacro.process_file(pathModelFile).toxml()


  #old way
  #urdf_filename = 'description/robot.urdf.xacro'

  #gz launch parameters
  # gazebo_package_name = 'ros_gz_sim' #This allows changing from classic, ignition, or gz
  # gazebo_launch_file_path = 'launch'
  # gazebo_models_folder = 'worlds'
  # gazebo_config_file = 'config/gz/ros_gz_bridge.yaml'
  # gazebo_config_folder = 'config/gz'  
  # gazebo_world_file = 'empty.world'    # e.g. 'world/empty.world', 'world/house.world'
  # gazebo_world_folder = 'worlds'
  

  gazebo_rosPackageLaunch=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),'launch','gz_sim.launch.py'))

  gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'gz_args': ['-r -v -v4 empty.sdf'],'on_exit_shutdown':'true'}.items())

  
  spawnModeNodeGazebo = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=['-topic', '/robot_description'
    ],
    output='screen', 
  )


  # spawnModeNodeGazebo = Node(
  #   package='ros_gz_sim',
  #   executable='create',
  #   arguments=[
  #     '-name', robot_name,
  #     '-topic', 'robot_description'
  #   ],
  #   output='screen', 
  # )

  # nodeRobotStatePublisher = Node(
  #   package='robot_state_publisher',
  #   executable='robot_state_publisher',
  #   output='screen',
  #   parameters=[{'robot_description': robotDescription,
  #                'use_sim_time': True}],
  # )

  bridge_params = os.path.join(
    get_package_share_directory(package_name),
    'config',
    'ros_gz_bridge.yaml'
  )


  start_gazebo_ros_bridge_cmd = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
      '--ros-args',
      '-p',
      f'config_file:={bridge_params}',
    ],
    output='screen',
  )

  launchDescriptionObject = LaunchDescription()

  launchDescriptionObject.add_action(gazeboLaunch)
  launchDescriptionObject.add_action(spawnModeNodeGazebo)
  # launchDescriptionObject.add_action(nodeRobotStatePublisher)
  launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)
  
  return launchDescriptionObject