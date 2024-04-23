import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():

    # Our purpose here is just to launch Gazebo and load our robot.
    # Note that we won't have any controllers yet.
    # We will load them in another launch script.  Therefore,
    # we cannot move our robot yet

    # pkg_path = os.path.join(get_package_share_directory('inmoov_base'))

    package_name='inmoov_base'

    # Here we just setup the Gazebo sim environment so we don't have to do it later.
    # This is not critical to the robot itself but it makes the sim easier to use or run better.
    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the EXTERNAL gazebo_ros package
    # Their launch file work perfectly fine.  No need to reinvent the wheel here.
    # At this point we should have an empty simulation world
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )

    # Run the spawner node from the EXTERNAL gazebo_ros package. 
    # The entity name doesn't really matter if you only have a single robot.
    # This is JUST the robot.  At some point we need some sanity code to ensure
    # Gazebo is loaded before we trigger this
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')


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
        gazebo,
        spawn_entity,
    ])

# Next, launch controllers.launch.py
