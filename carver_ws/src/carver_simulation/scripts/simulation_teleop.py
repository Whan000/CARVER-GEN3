#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys, select, termios, tty
import threading
import math

class CarverTeleop(Node):
    def __init__(self):
        super().__init__('carver_teleop')
        
        self.declare_parameter('mode', 'carver')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value

        self.steering_pub = self.create_publisher(Float64MultiArray, '/steering_controller/commands', 10)
        self.wheel_pub = self.create_publisher(Float64MultiArray, '/wheel_velocity_controller/commands', 10)
        
        self.max_velocity = 50.0
        self.max_steering_angle = math.radians(60)

        if self.mode == 'carver':
            self.velocity_step = 1.0
            self.steering_step = math.radians(5)
        else:
            self.velocity_step = -1.0
            self.steering_step = -math.radians(5)
        
        self.current_velocity = 0.0
        self.current_steering = 0.0
        self.running = True
        
        self.lock = threading.Lock()
        
        try:
            self.settings = termios.tcgetattr(sys.stdin)
            self.interactive_mode = True
        except:
            self.interactive_mode = False
            self.get_logger().warn('Cannot get terminal settings')
        
        self.timer = self.create_timer(0.05, self.publish_commands)
        
        self.get_logger().info('Carver Teleop Node initialized')
        self.get_logger().info(f'Max velocity: {self.max_velocity} m/s')
        self.get_logger().info(f'Max steering: {math.degrees(self.max_steering_angle)} degrees')
        
        if self.interactive_mode:
            self.print_controls()
        
    def print_controls(self):
        print('\n=== Carver Teleop Controls ===')
        print('  w/s: forward/backward')
        print('  a/d: left/right')
        print('  space: stop')
        print('  q: quit')
        print('===============================\n')
    
    def getKey(self):
        if not self.interactive_mode:
            return ''
            
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            return key
        except:
            return ''
    
    def publish_commands(self):
        with self.lock:
            steering_msg = Float64MultiArray()
            steering_msg.data = [self.current_steering, self.current_steering]
            self.steering_pub.publish(steering_msg)
            
            wheel_msg = Float64MultiArray()
            wheel_msg.data = [self.current_velocity, self.current_velocity]
            self.wheel_pub.publish(wheel_msg)
    
    def send_stop_commands(self):
        with self.lock:
            self.current_velocity = 0.0
            self.current_steering = 0.0
            
        steering_msg = Float64MultiArray()
        steering_msg.data = [0.0, 0.0]
        self.steering_pub.publish(steering_msg)
        
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [0.0, 0.0]
        self.wheel_pub.publish(wheel_msg)
        
        print('\nStop commands sent')
    
    def keyboard_loop(self):
        if not self.interactive_mode:
            return
            
        try:
            while self.running and rclpy.ok():
                key = self.getKey()

                if not key:
                    continue

                with self.lock:
                    if key == 'w':
                        self.current_velocity = min(
                            self.current_velocity + self.velocity_step,
                            self.max_velocity
                        )
                    elif key == 's':
                        self.current_velocity = max(
                            self.current_velocity - self.velocity_step,
                            -self.max_velocity
                        )
                    elif key == 'd':
                        self.current_steering = min(
                            self.current_steering + self.steering_step,
                            self.max_steering_angle
                        )
                    elif key == 'a':
                        self.current_steering = max(
                            self.current_steering - self.steering_step,
                            -self.max_steering_angle
                        )
                    elif key == ' ':
                        self.current_velocity = 0.0
                        self.current_steering = 0.0
                        print('\nStopped')
                    elif key == 'q':
                        print('\nQuit requested...')
                        self.running = False
                        break
                    elif key == '\x03':
                        self.running = False
                        raise KeyboardInterrupt
                        
                    print(f'\rVel: {self.current_velocity:6.2f} m/s | Steer: {math.degrees(self.current_steering):6.1f}°', 
                          end='', flush=True)
                        
        except KeyboardInterrupt:
            self.running = False
        except Exception as e:
            self.get_logger().error(f'Error in keyboard loop: {e}')
            self.running = False
        finally:
            if self.interactive_mode:
                try:
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
                except:
                    pass
    
    def run(self):
        keyboard_thread = None
        if self.interactive_mode:
            keyboard_thread = threading.Thread(target=self.keyboard_loop)
            keyboard_thread.daemon = True
            keyboard_thread.start()
        
        try:
            while self.running and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                
        except KeyboardInterrupt:
            print('\nKeyboard interrupt detected')
        except Exception as e:
            self.get_logger().error(f'Error in main loop: {e}')
        finally:
            self.running = False
            self.send_stop_commands()
            if keyboard_thread and keyboard_thread.is_alive():
                keyboard_thread.join(timeout=1.0)
            if self.interactive_mode:
                try:
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
                except:
                    pass
            print('\nCarver Teleop stopped')
    
    def destroy_node(self):
        self.running = False
        self.send_stop_commands()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CarverTeleop()
        node.run()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt - shutting down")
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()