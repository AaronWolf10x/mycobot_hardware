# mycobot_hardware

ros2_control hardware interface (C++) for the myCobot 280, with swappable drivers:

- `mock`: simulated arm, no hardware needed (default)
- `serial`: myCobot serial protocol. **Untested on real hardware. Verify the frame bytes first.**

## Requirements
- ROS 2 Lyrical with `ros2_control` and `ros2_controllers`
- `mycobot_description` from https://github.com/elephantrobotics/mycobot_ros2 (branch `humble`)

## Build and run
    cd ~/ros2_workspace/src
    git clone https://github.com/elephantrobotics/mycobot_ros2.git -b humble
    cp -r mycobot_ros2/mycobot_description .
    git clone <this repo url>
    cd ~/ros2_workspace
    colcon build --packages-select mycobot_description mycobot_hardware
    source install/setup.bash
    ros2 launch mycobot_hardware bringup.launch.py

## Move the arm
    ros2 topic pub --times 5 -r 2 /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: [joint2_to_joint1, joint3_to_joint2, joint4_to_joint3, joint5_to_joint4, joint6_to_joint5, joint6output_to_joint6], points: [{positions: [0.8, -0.5, 0.5, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}]}"
