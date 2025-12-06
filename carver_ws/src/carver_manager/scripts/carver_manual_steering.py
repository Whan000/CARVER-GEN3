#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, UInt16, Int8
from std_srvs.srv import SetBool


class CarverManualSteering(Node):
    def __init__(self):
        super().__init__('carver_manual_steering')
        self.declare_parameter('max_speed', 3.5) #m/s
        self.declare_parameter('adc_min', 700)
        self.declare_parameter('adc_max', 2949)
        self.declare_parameter('update_rate', 100.0) #hz naja
        self.declare_parameter('start_enabled', True)  # enabled by default
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.adc_min = self.get_parameter('adc_min').get_parameter_value().integer_value
        self.adc_max = self.get_parameter('adc_max').get_parameter_value().integer_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.publishing_enabled = self.get_parameter('start_enabled').get_parameter_value().bool_value
        self.current_adc = 0
        self.current_direction = 0
        self.speed_pub = self.create_publisher(Float32, '/manual_speed', 10)
        self.accel_sub = self.create_subscription(UInt16, '/accl_publisher', self.accel_callback, 10)
        self.direction_sub = self.create_subscription(Int8, '/accel_direction', self.direction_callback, 10)

        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self.compute_and_publish)
        
        self.get_logger().info('===== Carver Manual Steering Node =====')
        self.get_logger().info(f'Max Speed: {self.max_speed} m/s')
        self.get_logger().info(f'ADC Range: [{self.adc_min} - {self.adc_max}]')
        self.get_logger().info(f'Update Rate: {self.update_rate} Hz')
        self.get_logger().info(f'Publishing Enabled: {self.publishing_enabled}')
        self.get_logger().info('Service: /manual/enable')
    
    def accel_callback(self, msg):
        self.current_adc = msg.data
    
    def direction_callback(self, msg):
        """Callback for direction: 1=forward, -1=backward, 0=neutral"""
        self.current_direction = msg.data
    
    def enable_callback(self, request, response):
        self.publishing_enabled = request.data
        
        if self.publishing_enabled:
            response.success = True
            response.message = "Manual steering publishing ENABLED"
            self.get_logger().info("Publishing ENABLED")
        else:
            response.success = True
            response.message = "Manual steering publishing DISABLED"
            self.get_logger().info("Publishing DISABLED")
            
            # msg = Float32()
            # msg.data = 0.0/
            # self.speed_pub.publish(msg)
        
        return response
    
    def compute_and_publish(self):
        
        if not self.publishing_enabled:
            return
        
        adc_clamped = max(min(self.current_adc, self.adc_max), self.adc_min)
        
        adc_range = self.adc_max - self.adc_min
        if adc_range > 0:
            normalized = (adc_clamped - self.adc_min) / adc_range
        else:
            normalized = 0.0
        velocity = normalized * self.max_speed
        target_speed = velocity * self.current_direction
        
        msg = Float32()
        msg.data = target_speed
        # print(f"Publishing Target Speed: {target_speed:.2f} m/s")
        self.speed_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CarverManualSteering()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt - shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()