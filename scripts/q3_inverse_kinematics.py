#!/usr/bin/env python3

# pyaaro Shebang


import rclpy  # the core Python ROS 2 library
from rclpy.node import Node  # class from rclpy.node, lets us define our custom node
from geometry_msgs.msg import Point  # message with x, y, z floats, used for /end_effector_position
from std_msgs.msg import Float64MultiArray  # message for sending a list of floats, used for the joint angles [θ₁, θ₂]
import math  # for pi, sin, cos, sqrt, 2D Inverse Kinematics may lgega ye sab

class InverseKinematicsNode(Node):  # our ROS 2 Node class

    def __init__(self):  
        super().__init__('inverse_kinematics_node')
        # our constructor, called when our Node is created

        self.L1 = 2.0  #(given)
        self.L2 = 1.5  #(given)
        self.max_step = 0.5  #don't let the user move more than 0.5 m at a time


        self.current_position = Point()  # to hold the latest received end-effector position: x, y


        self.sub = self.create_subscription(Point, '/end_effector_position', self.ee_callback, 10)
        # message type, subscribes (listens) to /end_effector_position, function that gets called whenever a message arrives, queue size


        self.pub = self.create_publisher(Float64MultiArray, '/joint_angles_goal', 10)
        # message type : list of floats (θ₁, θ₂), publishes to /joint_angles_goal, queue size


        # timer to prompt user input every 5 seconds
        self.timer = self.create_timer(5.0, self.get_user_input)

    def ee_callback(self, msg):
        self.current_position = msg  # when a new message arrives on /end_effector_position, save it


    def get_user_input(self):
        direction = input("Enter direction to move ('x' or 'y'): ").strip().lower()
                                                         # to remove spaces, toLowerCase


        if direction not in ['x', 'y']:  # garbage input check (I like to call it garbage input okay coz invalid hai what the duck user) :  invalid direction, print warning and exit early
            self.get_logger().warn("Invalid direction! Use 'x' or 'y'.")
            return

        try:
            distance = float(input("Enter distance (max 0.5 m): "))
        except ValueError:  # mtlb ki number na daalkr kuchh aur likhdia toh (afaiu - asfarasIunderstood)
            self.get_logger().warn("Invalid distance! Please enter a number.")
            return

        if abs(distance) > self.max_step:
            self.get_logger().warn("Max movement is 0.5 meters at a time.")  # given restriction, hence limiting movement
            return
        
        # toh ye sab toh ho gaye ji checks


        # computing new target
        target_x = self.current_position.x
        target_y = self.current_position.y

        if direction == 'x':
            target_x += distance
        else:
            target_y += distance



        if not self.is_reachable(target_x, target_y):  # haan isko aage kara hai define, chalte raho
            self.get_logger().warn("Target position unreachable!")
            return
        
        # if the new point is too far for the arm to reach, skip (skippiiiiii)



        # computing joint angles
        theta1, theta2 = self.compute_ik(target_x, target_y)  # ye bhi aage kara hai define
        # calculate angles from position using math


        # Publish
        msg = Float64MultiArray()
        msg.data = [theta1, theta2]  # pack the angles into a message
        self.pub.publish(msg)

        self.get_logger().info(f"Published joint angles: θ1 = {theta1:.2f}, θ2 = {theta2:.2f}")





    def is_reachable(self, x, y):
        r = math.sqrt(x**2 + y**2)
        return 0.5 <= r <= 3.5
    
    # LOGIC : distance from origin = √(x² + y²); the robotic arm can reach from 0.5 m (minimum bend) to 3.5 m (fully stretched)





     # classic 2D Inverse Kinematics

    def compute_ik(self, x, y):
        L1, L2 = self.L1, self.L2
        r2 = x**2 + y**2
        cos_theta2 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)
        theta2 = math.acos(cos_theta2)


        k1 = L1 + L2 * math.cos(theta2)
        k2 = L2 * math.sin(theta2)
        theta1 = math.atan2(y, x) - math.atan2(k2, k1)



        return theta1, theta2



 

def main(args=None):
    rclpy.init(args=args)  # starting ROS
    node = InverseKinematicsNode()  # creating the node
    rclpy.spin(node)  # to keep it running (spinning)
    node.destroy_node()  # ctrl + c (force stop) ko achhe se handle krne ke liye
    rclpy.shutdown()  # shutdown


# this makes sure our script runs only when executed, not when imported
if __name__ == '__main__':
    main()


# hum satya vachan bolenge, isme chatgpt ki sahaayata li gayi hai, uske bina nhi ho paata bhyi
# but fun learning (*cries)

