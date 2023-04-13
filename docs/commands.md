# Launch teleop_twist_keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard 

# Launch teleop_twist_joy
# See Here: https://index.ros.org/p/teleop_twist_joy/github-ros2-teleop_twist_joy/
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='ps3'

# Launch rViz2 with configuration
rviz2 -d inmoov_base/config/rviz.config.rviz

# Launch rViz2 with just a robot description to test the URDF design
ros2 launch inmoov_base rsp.launch.py



# Launch the robot_state_publisher / gazebo sim with a predefined world
ros2 launch inmoov_base launch_sim.launch.py world:=./dev_ws/src/inmoov_base/worlds/cafe.world 



# Launch rViz2 with a launch file
# https://answers.ros.org/question/374926/ros2-how-to-launch-rviz2-with-config-file/
