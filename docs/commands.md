
# Whenever launching, do the following commands to monitor and see if it works.

ros2 run rqt_console rqt_console
ros2 node list
ros2 topic list
ros2 status
ros2 interface show rcl interfaces/msg/Log
ros2 topic echo /rosout             # Logs

# Current Startup: as of 2023/5/21
ros2 launch inmoov_base launch_micro_sim.launch.py

ros2 launch inmoov_base launch_sim.launch.py world:=./src/inmoov_base/worlds/obstacles.world 

# Launch Gazebo with world
ros2 launch gazebo_ros gazebo.launch.py world:=./src/inmoov_base/worlds/obstacles.world 


# Launch teleop_twist_keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard 

# Launch teleop_twist_joy
# See Here: https://index.ros.org/p/teleop_twist_joy/github-ros2-teleop_twist_joy/
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='ps3'


# Launch rViz2 with just a robot description to test the URDF design
# This will launch our robot description and publish the model to the /robot_description
# Only non moving fixed parts should show if rviz2 was to be launched here.
ros2 launch inmoov_base rsp.launch.py

# Launch joint_state_publisher_gui
# We need this to have the wheels show up because otherwise there is no state information for rviz to read.
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# Launch rViz2 with configuration
# We should be able to see our robot now.
rviz2 -d inmoov_base/config/rviz.config.rviz



# Launch the robot_state_publisher / gazebo sim with a predefined world
ros2 launch inmoov_base launch_sim.launch.py world:=./dev_ws/src/inmoov_base/worlds/cafe.world 

ros2 launch inmoov_base launch_sim.launch.py world:=inmoov_base/worlds/empty.world 



# Launch rViz2 with a launch file
# https://answers.ros.org/question/374926/ros2-how-to-launch-rviz2-with-config-file/




# Not sure what all this means yet but I don't want to keep typing it
ros2 run controller_manager spawner diff_cont

ros2 run controller_manager spawner joint_broad

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped







## VSCode Stuff
# Install vscode-icons
Press Ctrl-p in vscode
Enter: ext install vscode-icons-team.vscode-icons
Activate when prompted



# Good information on how to setup VSCode even though alot of it pertains to ROS1
https://erdalpekel.de/
