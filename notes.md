## Robot Package Template

This is a GitHub template. You can make your own copy by clicking the green "Use this template" button.

It is recommended that you keep the repo/package name the same, but if you do change it, ensure you do a "Find all" using your IDE (or the built-in GitHub IDE by hitting the `.` key) and rename all instances of `inmoov_base` to whatever your project's name is.

Note that each directory currently has at least one file in it to ensure that git tracks the files (and, consequently, that a fresh clone has direcctories present for CMake to find). These example files can be removed if required (and the directories can be removed if `CMakeLists.txt` is adjusted accordingly).

Change Log October 7, 2025

Commands

launch code . from the directory where youve downloaded all the modules

##### Launch Commands In Order #####


ros2 launch inmoov_base bringup.launch.py

ros2 launch inmoov_base joystick.launch.py

ros2 run controller_manager ros2_control_node

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066FFF515250898367102831-if02

rqt

ros2 control interfaces etc.

Joystick
ros2 run teleop-twist_joy

ros2 launch inmoov_base launch_gz_sim_only.launch.py

ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.1}}"
ros2 topic pub /joint_states sensor_msgs/JointState "{name: ['joint1', 'joint2'], position: [1.0, 2.0], velocity: [0.5, 0.5], effort: [1.0, 1.5]}"



ros2 topic pub /cmd_vel geometry_msgs/msg/Twist




//			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//			printf("Wheel Speeds: %f,  %f \n",left_speed,right_speed);


HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);



```cpp
//	uint16_t pulse_width = (uint16_t)(((float)position / 180.0) * (SERVO_MAX_PULSE_WIDTH - SERVO_MIN_PULSE_WIDTH) + SERVO_MIN_PULSE_WIDTH);
//	printf ("%d", position);
	TIM2->CCR1 = position;
//	TIM2->ARR = something;
//	int converter = joint_states_position_buffer;
//	joint_states_msg.position.data
//	TIM2->CCR1 = converter+75;
//	osDelay(1000);
//	TIM2->CCR1 = converter *1;
//	osDelay(50);
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
```







	
  /**********************************************************
   * We can have multiple subscriptions for each node.
   * So we will create a joint_states_subscriber and
   * a cmd_vel_joy subscriber
   **********************************************************/

	rclc_subscription_init_default(
		&joint_states_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"/cmd_vel_joy");



	// create joint_state subscriber
	/* We need to start our subscriber.
	 * &joint_statessssssssssss_subscriber seems to be a pointer de-ref to somewhere
	 * that I did not initialize.  Maybe this is why I am not seeing any data
	 * Or maybe it is part of something else that gets initalized.
	 *
	 * We initialize our subscriber by specifying which node it will be
	 * attached to and what type of messages are coming along.
	 * I believe that the parameters are a pre-defined standard part of the
	 * ROSIDL (ROS2 Interface Definition Language)
	 * https://design.ros2.org/articles/idl_interface_definition.html
	 */
	rclc_subscription_init_default(
		&joint_states_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
		"/joint_states");

/****************************************************************/




    # pdb.set_trace()  #Python debug break if needed
