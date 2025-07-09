# Week 4A Assignment : Robotic Arm 

## Q1: Forward Kinematics Node

This ROS 2 node performs forward kinematics for a simple 2D, 2-link robotic arm.

### Problem Statement

- The robot has:
  - Two revolute joints:
    - **shoulder_pitch_joint** (θ₁)
    - **elbow_pitch_joint** (θ₂)
  - Link lengths (given):
    - L₁ = 2.0 m
    - L₂ = 1.5 m

- Our GOAL is to :
  - subscribe to joint angles on `/joint_states`
  - compute 2D end-effector position (x, y)
  - publish it on `/end_effector_position` using `geometry_msgs/msg/Point`




### Node Information


 `/joint_states` : `sensor_msgs/msg/JointState` - Subscribes joint angles 
 `/end_effector_position` : `geometry_msgs/msg/Point` - Publishes computed (x, y) position


**also note that shoulder pitch joint ka reference is -90degrees
**toh we add pi/2 to that angle to obtain our required theta 1 (theta 2 is fine as entered)


### Code lines that we run in terminal to execute and run this

1. to build and source the workspace

cd ~/ros2_ws
colcon build --packages-select week4_arm
source install/setup.bash

2. to launch the robot model in RViz
ros2 launch week4_arm display_arm.launch.py

3. to run the forward kinematics node
ros2 run week4_arm q1_forward_kinematics.py

4. to run the GUI to adjust joint angles
ros2 run joint_state_publisher_gui joint_state_publisher_gui

5. to observe the output
ros2 topic echo /end_effector_position

[Q1 Robotic Arm working video (RIVZ) (Screenrecorded) watch](media/rivz_working_video_Week4A_Kratos.mp4)
