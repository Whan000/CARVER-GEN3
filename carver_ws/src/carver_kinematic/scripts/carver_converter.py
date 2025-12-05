#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32
import numpy as np

class ConverterNode(Node):
    def __init__(self):
        super().__init__('ConverterNode')
        
        self.declare_parameter('max_steering_angle', 0.6)
        self.max_steering_angle = self.get_parameter('max_steering_angle').value

        self.create_subscription(AckermannDrive, "/ackermann_cmd", self.ackermann_cmd_callback, 10)
        
        self.target_accel_pub = self.create_publisher(Float32, "/target_accel", 10)
        self.steering_angle_pub = self.create_publisher(Float32, "/steering_angle", 10)
        
    def ackermann_cmd_callback(self, msg: AckermannDrive):
        steering_msg = Float32()
        steering_msg.data = float(np.clip(msg.steering_angle, -self.max_steering_angle, self.max_steering_angle))  # Fixed: clip both directions
        self.steering_angle_pub.publish(steering_msg)
        
        # Publish acceleration
        accel_msg = Float32()
        accel_msg.data = msg.acceleration
        self.target_accel_pub.publish(accel_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = ConverterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()