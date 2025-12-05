#!/usr/bin/python3

import odrive
from odrive.enums import *
from odrive.enums import AXIS_STATE_UNDEFINED, AXIS_STATE_CLOSED_LOOP_CONTROL
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import Float32


class CarverOdriveNode(Node):
    def __init__(self):
        super().__init__('carver_odrive_node')
        
        self.declare_parameter('control_mode', 'velocity')
        self.declare_parameter('motor_inertia', 0.1)
        self.declare_parameter('wheel_radius', 0.175)
        self.declare_parameter('current_limit', 50.0)
        self.declare_parameter('max_velocity_m_s', 3.5)
        self.declare_parameter('max_acceleration_m_s2', 10.0)
        self.declare_parameter('torque_constant', 0.48)
        
        self.control_mode = self.get_parameter('control_mode').value
        self.motor_inertia = self.get_parameter('motor_inertia').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.current_limit = self.get_parameter('current_limit').value
        self.max_velocity_m_s = self.get_parameter('max_velocity_m_s').value * 10.0
        self.max_acceleration_m_s2 = self.get_parameter('max_acceleration_m_s2').value
        self.torque_constant = self.get_parameter('torque_constant').value
        
        self.max_acceleration_rad = self.max_acceleration_m_s2 / self.wheel_radius
        self.max_torque = self.motor_inertia * self.max_acceleration_rad
        
        if self.control_mode == 'velocity':
            self.create_subscription(Float32, "/target_speed", self.speed_callback, 10)
        else:
            self.create_subscription(Float32, "/target_accel", self.accel_callback, 10)
            
        self.wheel_velocity_pub = self.create_publisher(Float32, "feedback_wheelspeed", 10)
        self.torque_pub = self.create_publisher(Float32, "feedback_torque", 10)
        self.acceleration_pub = self.create_publisher(Float32, "feedback_acceleration", 10)
        
        self.create_timer(0.01, self.odrive_loop)
        
        self.target_velocity_rad = 0.0
        self.target_acceleration_rad = 0.0
        
        self.get_logger().info(f"Starting in {self.control_mode.upper()} control mode")
        self.get_logger().info(f"Wheel radius: {self.wheel_radius} m")
        self.get_logger().info(f"Motor inertia: {self.motor_inertia} kg·m²")
        self.get_logger().info(f"Torque constant: {self.torque_constant} Nm/A")
        self.get_logger().info(f"Max velocity: {self.max_velocity_m_s} m/s")
        self.get_logger().info(f"Max acceleration: {self.max_acceleration_m_s2} m/s² ({self.max_acceleration_rad:.2f} rad/s²)")
        self.get_logger().info(f"Max torque (calculated): {self.max_torque:.2f} Nm")
        self.initial_odrive()

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
                torque_cmd = self.motor_inertia * self.target_acceleration_rad
                torque_cmd = max(-self.max_torque, min(self.max_torque, torque_cmd))
                
                axis.controller.config.control_mode = ControlMode.TORQUE_CONTROL
                axis.controller.input_torque = torque_cmd
                
            else:
                vel_rev_s = self.target_velocity_rad / (2.0 * math.pi)
                
                axis.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
                axis.controller.input_vel = vel_rev_s

            actual_velocity_rev_s = axis.vel_estimate
            # print(f"ODrive reported velocity: {actual_velocity_rev_s} rev/s")
            actual_velocity_rad_s = actual_velocity_rev_s * 2.0 * math.pi
            actual_velocity_m_s = actual_velocity_rad_s * self.wheel_radius
            
            actual_current_iq = axis.motor.foc.Iq_measured
                                
            actual_torque = actual_current_iq * self.torque_constant
            
            actual_acceleration_rad_s2 = actual_torque / self.motor_inertia
            actual_acceleration_m_s2 = actual_acceleration_rad_s2 * self.wheel_radius
            
            wheel_velocity_msg = Float32()
            wheel_velocity_msg.data = float(actual_velocity_m_s)
            self.wheel_velocity_pub.publish(wheel_velocity_msg)
            
            torque_msg = Float32()
            torque_msg.data = float(actual_torque)
            self.torque_pub.publish(torque_msg)
            
            accel_msg = Float32()
            accel_msg.data = float(actual_acceleration_m_s2)
            self.acceleration_pub.publish(accel_msg)
            
            # self.get_logger().info(f"Speed: {actual_velocity_m_s:.2f} m/s, Torque: {actual_torque:.2f} Nm, Accel: {actual_acceleration_m_s2:.2f} m/s²")

        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")
            self.get_logger().info("Attempting to reconnect to ODrive...")
            try:
                self.initial_odrive()
                self.get_logger().info("Reconnected to ODrive.")
            except Exception as reconnection_error:
                self.get_logger().error(f"Failed to reconnect to ODrive: {reconnection_error}")
    
    def speed_callback(self, msg: Float32):
        speed_m_s = msg.data * 10.0
        speed_m_s = max(-self.max_velocity_m_s, min(self.max_velocity_m_s, speed_m_s))
        self.target_velocity_rad = speed_m_s / self.wheel_radius
        # self.get_logger().info(f"Received speed: {speed_m_s:.2f} m/s → {self.target_velocity_rad:.2f} rad/s")
    
    def accel_callback(self, msg: Float32):
        accel_m_s2 = msg.data
        accel_m_s2 = max(-self.max_acceleration_m_s2, min(self.max_acceleration_m_s2, accel_m_s2))
        self.target_acceleration_rad = accel_m_s2 / self.wheel_radius
        # self.get_logger().info(f"Received accel: {accel_m_s2:.2f} m/s² → {self.target_acceleration_rad:.2f} rad/s²")

def main(args=None):
    rclpy.init(args=args)
    node = CarverOdriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()