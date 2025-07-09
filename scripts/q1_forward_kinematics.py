#!/usr/bin/env python3

# mhaaro pyaaro Shebang



import rclpy  # main Python client library for ROS 2
from rclpy.node import Node  # base class for creating ROS 2 nodes
from sensor_msgs.msg import JointState  # message type that gives joint angles
from geometry_msgs.msg import Point  # message type that holds x, y, z positions
import math  # ye pi and sin, cos functions ke liye chahiye 




class ForwardKinematicsNode(Node):



    def __init__(self):  #constructor
        super().__init__('forward_kinematics_node')  #super refers the parent class, which is "Node" here;
                                                     #init calls the constructor of the parent class; 
                                                     #while forward_kineatics_node is the name of our ROS2 Node

        # Link lengths (given in question)
        self.L1 = 2.0
        self.L2 = 1.5

        # Subscriber to joint states
        self.subscription = self.create_subscription( JointState, '/joint_states', self.joint_callback, 10 ) 
                                                    # message type; topic; when a message comes, call; queue size 

        # Publisher for end-effector position
        self.publisher_ = self.create_publisher(Point, '/end_effector_position', 10)
                                    # type 'geometry_msgs/msg/Point', topic, queue size
        self.get_logger().info("Forward Kinematics Node started")
          # this "logs" a message to the terminal so we know that the node has started





    def joint_callback(self, msg):  
        # this function gets called every time a new /joint_states message arrives (self.create subscription may likha hai see),
        # defining this function here now
        
        if len(msg.position) < 2:
            self.get_logger().warn("Not enough joint data received")
            return
        
        #  Safety check: if the joint state message doesn’t contain 2 positions, it skips the calculation; minimum 2 position are required cmon


        theta1 = msg.position[0] + math.pi/2  # to adjust shoulder joint (given in question) # shoulder joint
        theta2 = msg.position[1] # elbow joint

        x = self.L1 * math.cos(theta1) + self.L2 * math.cos(theta1 + theta2)
        y = self.L1 * math.sin(theta1) + self.L2 * math.sin(theta1 + theta2)

        point = Point()  #creates a 'Point' message
        point.x = x
        point.y = y
        point.z = 0.0  # not used coz 2D hai ye question


        self.publisher_.publish(point)
        self.get_logger().info(f"Published End-Effector Position: x={x:.2f}, y={y:.2f}")
        # the fstring helps insert variables within the string; {} ke andrr jo likha, vo ho jaayega aapka vo variable; .2f -> to format it to 2 decimal places





def main(args=None):  # the main entry point
    rclpy.init(args=args)  # initialises ROS2
    node = ForwardKinematicsNode()  # creates the Node
    rclpy.spin(node)  # keeps it alive (spins)
    node.destroy_node()  # ctrl + c (force stop) ko achhe se handle krne ke liye
    rclpy.shutdown()  # as the name suggests, to shutdown 



# this makes sure our script runs only when executed, not when imported
if __name__ == '__main__':
    main()
