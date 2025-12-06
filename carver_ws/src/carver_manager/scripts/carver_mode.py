#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Float32
from std_srvs.srv import SetBool


class CarverMode(Node):
    def __init__(self):
        super().__init__('carver_mode')
        
        self.MODE_MANUAL = 0
        self.MODE_TELEOP = 1
        self.MODE_AUTO = 2
        self.MODE_JOYSTICK = 3
        
        self.current_mode = None
        self.controller_speed = 0.0
        self.manual_speed = 0.0
    
        self.speed_pub = self.create_publisher(Float32, '/target_speed', 10)
        self.mode_sub = self.create_subscription(Int8,'/carver_mode', self.mode_callback,10)
        self.controller_sub = self.create_subscription(Float32,'/controller_speed', self.controller_callback,10)
        self.manual_sub = self.create_subscription(Float32,'/manual_speed', self.manual_callback,10)
        
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        self.get_logger().info('===== Carver Mode Switching Node =====')
        self.get_logger().info('Modes: 0=MANUAL, 1=TELEOP, 2=AUTO, 3=JOYSTICK')
        self.get_logger().info('DEFAULT LOCK-ON MANUAL')
        self.get_logger().info('Ready to switch modes')
    
    def mode_callback(self, msg):
        new_mode = msg.data
        
        if new_mode == self.current_mode:
            return
        
        if new_mode not in [self.MODE_MANUAL, self.MODE_TELEOP, self.MODE_AUTO, self.MODE_JOYSTICK]:
            self.get_logger().debug(f'Ignoring invalid mode: {new_mode}')
            return
        
        mode_name = self.get_mode_name(new_mode)
        prev_mode_name = self.get_mode_name(self.current_mode)
        
        self.get_logger().info(f'Mode change: {prev_mode_name} → {mode_name}')
    
        self.current_mode = new_mode

    def controller_callback(self, msg):
        self.controller_speed = msg.data

    def manual_callback(self, msg):
        self.manual_speed = msg.data
    
    def timer_callback(self):
        """Publish speed based on current mode"""
        msg = Float32()
        
        if self.current_mode == self.MODE_MANUAL:
            self.get_logger().debug('In MANUAL mode - speed from /manual_speed topic')
            msg.data = self.manual_speed
        elif self.current_mode == self.MODE_AUTO:
            self.get_logger().debug('In AUTO mode - speed from /controller_speed topic')
            msg.data = self.controller_speed
        
        self.speed_pub.publish(msg)

    def get_mode_name(self, mode):
        """Get human-readable mode name"""
        if mode == self.MODE_MANUAL:
            return 'MANUAL'
        elif mode == self.MODE_TELEOP:
            return 'TELEOP'
        elif mode == self.MODE_AUTO:
            return 'AUTO'
        elif mode == self.MODE_JOYSTICK:
            return 'JOYSTICK'
        elif mode is None:
            return 'NONE'
        else:
            return f'UNKNOWN({mode})'


def main(args=None):
    rclpy.init(args=args)
    node = CarverMode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt - shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()