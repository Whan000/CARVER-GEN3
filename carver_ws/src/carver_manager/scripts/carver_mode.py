#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from std_srvs.srv import SetBool


class CarverMode(Node):
    def __init__(self):
        super().__init__('carver_mode')
        
        self.MODE_MANUAL = 0
        self.MODE_TELEOP = 1
        self.MODE_AUTO = 2
        self.MODE_JOYSTICK = 3
        
        self.current_mode = None
    
        self.mode_sub = self.create_subscription(Int8,'/carver_mode',self.mode_callback,10)

        self.manual_enable_client = self.create_client(SetBool, '/manual/enable')
        self.teleop_enable_client = self.create_client(SetBool, '/teleop/enable')
        self.auto_enable_client = self.create_client(SetBool, '/auto/enable')
        self.joystick_enable_client = self.create_client(SetBool, '/joystick/enable')
        
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
        
        self.switch_mode(new_mode)
        self.current_mode = new_mode
    
    def switch_mode(self, mode):
        """Enable selected mode, disable all others"""
        
        enable_manual = (mode == self.MODE_MANUAL)
        enable_teleop = (mode == self.MODE_TELEOP)
        enable_auto = (mode == self.MODE_AUTO)
        enable_joystick = (mode == self.MODE_JOYSTICK)
        self.call_enable_service(self.manual_enable_client, 'MANUAL', enable_manual)
        self.call_enable_service(self.teleop_enable_client, 'TELEOP', enable_teleop)
        self.call_enable_service(self.auto_enable_client, 'AUTO', enable_auto)
        self.call_enable_service(self.joystick_enable_client, 'JOYSTICK', enable_joystick)
        
        self.get_logger().info(f'Switched to {self.get_mode_name(mode)} mode')
    
    def call_enable_service(self, client, mode_name, enable):
        """Call enable service if available"""
        
        if not client.service_is_ready():
            self.get_logger().debug(f'{mode_name} service not available - skipping')
            return
        
        request = SetBool.Request()
        request.data = enable
        
        future = client.call_async(request)
        future.add_done_callback(
            lambda f: self.service_response_callback(f, mode_name, enable)
        )
    
    def service_response_callback(self, future, mode_name, enable):
        """Handle service response"""
        try:
            response = future.result()
            status = "ENABLED" if enable else "DISABLED"
            if response.success:
                self.get_logger().debug(f'{mode_name} {status}: {response.message}')
            else:
                self.get_logger().warn(f'Failed to change {mode_name}: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed for {mode_name}: {e}')
    
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