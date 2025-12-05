#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf_transformations import euler_from_quaternion

class BnoPublisher(Node):
    def __init__(self):
        super().__init__('bno055_imu_node')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.raw_data = Float64MultiArray()
        self.create_subscription(Float64MultiArray,'/bno055_data',self.imu_converter_callback,qos_profile)
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)
        self.create_timer(0.01, self.timer_callback)
        self.get_logger().info('BNO055 remapping node has been started.')
            
    def imu_converter_callback(self, msg):  
        self.raw_data = msg
        
    def timer_callback(self):
        if len(self.raw_data.data) < 16:
            return
            
        imu_msg = Imu()
        imu_msg.header.frame_id = "IMU_L_Link"
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        
        imu_msg.orientation.x = self.raw_data.data[0]
        imu_msg.orientation.y = self.raw_data.data[1]
        imu_msg.orientation.z = self.raw_data.data[2]
        imu_msg.orientation.w = self.raw_data.data[3]
        
        imu_msg.angular_velocity.x = self.raw_data.data[7]
        imu_msg.angular_velocity.y = self.raw_data.data[8]
        imu_msg.angular_velocity.z = self.raw_data.data[9]
        
        imu_msg.linear_acceleration.x = self.raw_data.data[4]
        imu_msg.linear_acceleration.y = self.raw_data.data[5]
        imu_msg.linear_acceleration.z = self.raw_data.data[6]
        
        imu_msg.orientation_covariance[0] = 0.01
        imu_msg.angular_velocity_covariance[0] = 0.01
        imu_msg.linear_acceleration_covariance[0] = 0.01
        
        self.imu_publisher.publish(imu_msg)
        
        roll, pitch, yaw = euler_from_quaternion([
            imu_msg.orientation.x,
            imu_msg.orientation.y,
            imu_msg.orientation.z,
            imu_msg.orientation.w
        ])
        
        # print(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")
        # print(f"Angular Velocity - x: {imu_msg.angular_velocity.x:.2f}, y: {imu_msg.angular_velocity.y:.2f}, z: {imu_msg.angular_velocity.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = BnoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
