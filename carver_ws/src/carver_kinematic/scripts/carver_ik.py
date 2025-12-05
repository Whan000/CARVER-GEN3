#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np

class IKNode(Node):
    def __init__(self):
        super().__init__('IKNode')
        
        self.wheelbase = 0.8  # m
        
        self.declare_parameter('max_v', 3.5)
        self.max_v = self.get_parameter('max_v').value
        self.declare_parameter('max_w', 0.855)
        self.max_w = self.get_parameter('max_w').value
        self.declare_parameter('max_steering_angle', 0.6)
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
    
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        
        self.target_speed_pub = self.create_publisher(Float32, "/target_speed", 10)
        self.steering_angle_pub = self.create_publisher(Float32, "/steering_angle", 10)
        
    def cmd_vel_callback(self, msg: Twist):
        v = np.clip(msg.linear.x, -self.max_v, self.max_v)
        w = np.clip(msg.angular.z, -self.max_w, self.max_w)
        
        speed_msg = Float32()
        speed_msg.data = float(v)
        self.target_speed_pub.publish(speed_msg)
        
        # Bicycle model: omega_z = (v / L) * tan(delta)
        # delta = arctan(L * omega_z / v)
        steering_msg = Float32()
        if abs(v) > 1e-6:  # Fixed: epsilon comparison
            steering_angle = np.arctan(self.wheelbase * w / v)  # Fixed: arctan not arctan2
            steering_msg.data = float(np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle))
        else:
            steering_msg.data = 0.0
            
        self.steering_angle_pub.publish(steering_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()