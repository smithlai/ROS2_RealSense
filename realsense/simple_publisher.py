import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
import math

import numpy as np # Scientific computing library for Python


class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(Quaternion, 'mytopic', 10)
        self.radians={
            "r" : 0,
            "y" : 0,
            "p" : 0,
        }
        self.ranges={
            "r" : [0,0,0],
            "y" : [0,180,15],
            "p" : [0,90,15],
        }

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.step("p")
        self.step("y")
        self.get_logger().info('Publishing1: "%s"' % f"{self.radians['p']}, {self.radians['r']},{self.radians['y']}")
        pitch = math.radians(self.radians["p"])
        roll = math.radians(self.radians['r'])
        yaw = math.radians(self.radians['y'])
        
        msg = quaternion_from_euler(roll, yaw, pitch)
        self.publisher_.publish(msg)
        # Display the message on the console
        self.get_logger().info('Publishing2: "%s"' % msg)
    
    def step(self,key):
        range = self.ranges[key]
        self.radians[key] = self.radians[key] + range[2]
        if self.radians[key] >= range[1]:
            range[2] = -range[2]
        if self.radians[key] <= range[0]:
            range[2] = range[2]
    
def quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  q = Quaternion()
  q.x = qx
  q.y = qy
  q.z = qz
  q.w = qw
  return q
         
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    simple_publisher = SimplePublisher()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(simple_publisher)
    # Explicity destroy the node
    simple_publisher.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()