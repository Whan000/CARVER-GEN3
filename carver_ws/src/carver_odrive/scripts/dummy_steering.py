#!/usr/bin/python3

import odrive
from odrive.enums import *
from odrive.enums import AXIS_STATE_UNDEFINED, AXIS_STATE_CLOSED_LOOP_CONTROL
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import Float32, UInt16, Int8
from numpy import interp


class CarverOdriveNode(Node):
    def __init__(self):
        super().__init__('carver_odrive_node')
        
        self.declare_parameter('control_mode', 'velocity') #torque or velocity
        self.declare_parameter('motor_inertia', 0.1) #0.0
        self.declare_parameter('max_torque', 30.0) #0.0
        self.declare_parameter('max_acceleration', 300.0) #377
        self.declare_parameter('max_velocity', 235.5)
        self.declare_parameter('current_limit', 50.0) #50
        
        self.control_mode = self.get_parameter('control_mode').value
        self.motor_inertia = self.get_parameter('motor_inertia').value
        self.max_torque = self.get_parameter('max_torque').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.current_limit = self.get_parameter('current_limit').value
        
        self.create_subscription(UInt16, "/accl_publisher", self.accl_callback, 10)
        self.create_subscription(Int8, "/accel_direction", self.accel_dir_callback, 10)
        self.wheel_velocity_pub = self.create_publisher(Float32, "feedback_wheelspeed", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        
        self.create_timer(0.01, self.odrive_loop)
        
        self.accl_vel = 0
        self.target_acceleration = 0.0
        self.accel_dir = 0
        
        self.get_logger().info(f"Starting in {self.control_mode.upper()} control mode")
        self.initial_odrive()
        
    def accel_dir_callback(self, msg: Int8):
        self.accel_dir = msg.data

    def Odrive_VelControl(self):
        self.odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        self.odrv.axis0.controller.config.vel_ramp_rate = 60
        self.odrv.axis0.controller.config.input_mode = InputMode.VEL_RAMP
        
    def Odrive_TorqueControl(self):
        self.odrv.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
        self.odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
        
    def initial_odrive(self):
        self.odrv = odrive.find_any()
        
        while self.odrv.axis0.current_state == AXIS_STATE_UNDEFINED:
            time.sleep(0.01)

        while self.odrv.axis0.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            self.odrv.clear_errors()
            self.odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            time.sleep(0.01)
        
        time.sleep(1)
        
        # FIX: Change motor.config to config.motor
        self.get_logger().info(f"Current soft max: {self.odrv.axis0.config.motor.current_soft_max} A")
        self.odrv.axis0.config.motor.current_soft_max = self.current_limit
        self.odrv.config.dc_max_positive_current = self.current_limit
        self.odrv.config.dc_max_negative_current = -self.current_limit
        self.get_logger().info(f"New current limit: {self.current_limit}A")
        
        if self.control_mode == 'torque':
            self.odrv.axis0.controller.config.inertia = self.motor_inertia
            self.get_logger().info(f"Set inertia: {self.motor_inertia} kg·m²")
            self.Odrive_TorqueControl()
            self.get_logger().info("TORQUE CONTROL MODE activated")
        else:
            self.Odrive_VelControl()
            self.get_logger().info("VELOCITY CONTROL MODE activated")
        
        print("Finished setup Odrive")
        time.sleep(1)
        
    def odrive_loop(self):
        try:
            axis = self.odrv.axis0
            
            if hasattr(axis, 'error') and (axis.error != 0 or 
                                        axis.motor.error != 0 or 
                                        axis.encoder.error != 0):
                self.get_logger().error("ODrive Axis Errors detected.")
                self.get_logger().error(f"Axis Error: {hex(axis.error)}")
                self.get_logger().error(f"Motor Error: {hex(axis.motor.error)}")
                self.get_logger().error(f"Encoder Error: {hex(axis.encoder.error)}")

                try:
                    self.odrv.clear_errors()
                    self.get_logger().info("ODrive errors cleared.")
                except Exception as clear_err:
                    self.get_logger().error(f"Failed to clear ODrive errors: {clear_err}")

                self.get_logger().info("Attempting to reconnect to ODrive due to detected errors...")
                try:
                    self.initial_odrive()
                    self.get_logger().info("Successfully reconnected to ODrive.")
                except Exception as reconnection_error:
                    self.get_logger().error(f"Failed to reconnect to ODrive: {reconnection_error}")

            if self.control_mode == 'torque':
                torque_cmd = self.motor_inertia * self.target_acceleration
                torque_cmd = max(-self.max_torque, min(self.max_torque, torque_cmd))
                
                axis.controller.config.control_mode = ControlMode.TORQUE_CONTROL
                axis.controller.input_torque = torque_cmd
                
                current_vel = axis.vel_estimate
                # self.get_logger().info(f"Accel: {self.target_acceleration:.2f} rad/s², Torque: {torque_cmd:.2f} Nm, Vel: {current_vel:.2f} rev/s")
                
            else:
                self.vx_speed = self.accl_vel / (2.0 * math.pi)
                
                axis.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
                axis.controller.input_vel = self.vx_speed
                
                # self.get_logger().info(f"Set Speed = {self.accl_vel:.2f} rad/s ({self.vx_speed:.2f} rev/s)")

            wheel_velocity = axis.pos_vel_mapper.vel * 2.0 * math.pi * 1.041677 / 9.65
            wheel_velocity_msg = Float32()
            wheel_velocity_msg.data = float(wheel_velocity * 0.16)
            self.wheel_velocity_pub.publish(wheel_velocity_msg)

        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")
            self.get_logger().info("Attempting to reconnect to ODrive...")
            try:
                self.initial_odrive()
                self.get_logger().info("Reconnected to ODrive.")
            except Exception as reconnection_error:
                self.get_logger().error(f"Failed to reconnect to ODrive: {reconnection_error}")
    
    def accl_callback(self, msg: UInt16):
        raw_data = msg.data
        if raw_data < 620:
            raw_data = 0
        
        if self.control_mode == 'torque':
            acceleration = self.accel_dir * float(interp(float(raw_data), [620, 3200], [0, self.max_acceleration]))
            self.target_acceleration = acceleration
            
            msg = Twist()
            msg.linear.x = self.target_acceleration
            self.cmd_vel_pub.publish(msg)
        else:
            linear_vel = self.accel_dir * float(interp(float(raw_data), [620, 3200], [0, self.max_velocity]))
            self.accl_vel = linear_vel
            
            msg = Twist()
            msg.linear.x = linear_vel
            self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CarverOdriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()