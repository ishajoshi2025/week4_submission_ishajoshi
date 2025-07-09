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

<<<<<<< HEAD
* this file was huge so umm had to use git lft but yeah, we ball 🏀
=======
# this file was huge so umm had to use git lft but yeah, we ball 🏀
>>>>>>> 06e99b7 (Add Q3 inverse kinematics node for interactive end-effector control)



-----------------------------------------------------------------------------------------

-----------------------------------------------------------------------------------------

# Q3 : Interactive Inverse Kinematics Control

## Overview

This ROS 2 node demonstrates **interactive inverse kinematics** for a 2D robotic arm with two revolute joints (shoulder and elbow). The arm is composed of two links:

* Link 1 (L1) = 2.0 meters
* Link 2 (L2) = 1.5 meters

The node:

* **Subscribes** to the current end-effector position (`/end_effector_position`)
* **Takes user input** to move the end-effector by up to 0.5 meters in the X or Y direction
* **Computes** inverse kinematics to determine the required joint angles  theta 1 and theta 2)
* **Publishes** these joint angles to the `/joint_angles_goal` topic using `Float64MultiArray`

The node ensures that unreachable target positions are detected and gracefully ignored.

---

## Logical Approach

1. **Input & Safety Checks**:

   * Direction is entered by the user (`x` or `y`).
   * Distance must be ≤ 0.5 meters.
   * Target position is calculated from current position.

2. **Reachability Check**:

   * The end-effector can only move between 0.5 m and 3.5 m from the origin.
   * If the new target is outside this range, the node warns and aborts.

3. **Inverse Kinematics Calculation**:

   * Using standard 2D IK equations:

     * Law of cosines is used to get theta 2
     * Then theta 1 is derived from geometry (triangle formed by links and target)
   * The node assumes the "elbow-down" solution.

4. **Publishing**:

   * The computed joint angles are published to `/joint_angles_goal`.
   * These can then be used by another system to move the robot.

---

## Code Explanation



 `class InverseKinematicsNode(Node):`          
  Defines the ROS 2 Node class                                                


 `self.L1 = 2.0; self.L2 = 1.5`                
  Set the lengths of the robotic arm's links                                  


 `self.sub = self.create_subscription(...)`    
  Subscribe to `/end_effector_position` to get current arm position           


 `self.pub = self.create_publisher(...)`       
  Publisher to send out new joint angles                                      


 `self.create_timer(5.0, self.get_user_input)` 
  Every 5 seconds, the node will request new user input                       


 `self.get_user_input()`                       
  Function that handles interactive input and the inverse kinematics pipeline 


 `is_reachable(x, y)`                          
  Checks whether the given (x, y) is within the reachable area of the robot   


 `compute_ik(x, y)`                            
  Computes inverse kinematics using law of cosines + triangle geometry        


 `Float64MultiArray()`                         
  Used to send a list of two float values  theta 1, theta 2)                  


 `self.get_logger().warn(...)`                 
  Used for printing ROS 2-style warning logs                                  


 `rclpy.spin(node)`                            
  Keeps the node alive and responsive to messages/timers                      


---

Example:

Enter direction to move ('x' or 'y'): x
Enter distance (max 0.5 m): 0.3


4. If the new position is reachable:

* Joint angles will be published to `/joint_angles_goal`.

---

## Published Topics


topic `/joint_angles_goal` : type `std_msgs/Float64MultiArray` - Published joint angles theta 1 and theta 2

---

## Subscribed Topics


topic `/end_effector_position` : type `geometry_msgs/Point` - Current end-effector position, expected from Q1 


---


## Edge Cases Handled

* Invalid direction (anything other than `x` or `y`)
* Distance input > 0.5 meters
* Target position out of reach

Each case prints a warning without crashing the node.

