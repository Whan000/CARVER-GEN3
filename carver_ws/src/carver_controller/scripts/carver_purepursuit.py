#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, Path 
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
import math
import yaml
import os
from tf_transformations import euler_from_quaternion


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Declare parameters
        self.declare_parameter('wheelbase', 0.8)  # Distance between front and rear axles (m)
        self.declare_parameter('max_steering_angle', 0.6)  # ~30 degrees in radians
        self.declare_parameter('min_lookahead', 3.0)  # Minimum look-ahead distance (m)
        self.declare_parameter('max_lookahead', 5.0)  # Maximum look-ahead distance (m) 5.0
        self.declare_parameter('lookahead_gain', 1.0)  # Gain for dynamic look-ahead
        self.declare_parameter('target_speed', 1.00)  # Target speed (m/s)
        self.declare_parameter('max_speed', 1.00)  # Maximum speed (m/s)
        self.declare_parameter('waypoint_file', os.path.expanduser('~/carver_ws/src/carver_controller/path/trajectory1000.yaml'))
        
        # Get parameters
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.min_lookahead = self.get_parameter('min_lookahead').value
        self.max_lookahead = self.get_parameter('max_lookahead').value
        self.lookahead_gain = self.get_parameter('lookahead_gain').value
        self.target_speed = self.get_parameter('target_speed').value
        self.max_speed = self.get_parameter('max_speed').value
        self.waypoint_file = self.get_parameter('waypoint_file').value
        self.turnoff = False
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )
        
        self.create_subscription(Odometry, "/state_estimator/pose", self.pose_callback, 10)
        
        # Publisher
        self.steering_pub = self.create_publisher(Float32, "/steering_angle", 10)
        self.speed_pub = self.create_publisher(Float32, "/controller_speed", 10)
        self.path_viz_pub = self.create_publisher(Path, "/path_visualization", 10)
        self.target_marker_pub = self.create_publisher(Marker, "/target_point", 10)

        self.create_service(SetBool, "/auto/enable", self.bool_service_callback)
        
        # State variables
        self.current_pose = None
        self.current_yaw = None
        self.current_velocity = 0.0
        self.waypoints = []
        self.current_waypoint_idx = 0
        
        # Load waypoints
        self.load_waypoints_from_yaml()
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        self.create_timer(1.0, self.publish_path_visualization)
        
        self.get_logger().info('Pure Pursuit Controller initialized')

    def bool_service_callback(self, request, response):
        self.controller_enabled = request.data
        response.success = True
        response.message = "Enabled" if self.controller_enabled else "Disabled"
        self.get_logger().info(response.message)
        if not self.controller_enabled:
            self.speed_pub.publish(Float32(data=0.0))
            self.steering_pub.publish(Float32(data=0.0))
        return response        
    
    def load_waypoints_from_yaml(self):
        """Load waypoints from YAML file"""
        try:
            # Expand home directory path
            yaml_path = os.path.expanduser(self.waypoint_file)
            
            with open(yaml_path, 'r') as file:
                data = yaml.safe_load(file)
            
            # Extract waypoints based on common YAML formats
            # Adjust this based on your actual YAML structure
            if 'waypoints' in data:
                waypoints_data = data['waypoints']
            elif 'path' in data:
                waypoints_data = data['path']
            elif 'poses' in data:
                waypoints_data = data['poses']
            else:
                # Assume the data itself is a list of waypoints
                waypoints_data = data
            
            # Convert to numpy array
            waypoint_list = []
            for wp in waypoints_data:
                if isinstance(wp, dict):
                    # Handle dictionary format: {x: val, y: val} or {pose: {position: {x: val, y: val}}}
                    if 'x' in wp and 'y' in wp:
                        waypoint_list.append([wp['x'], wp['y']])
                    elif 'pose' in wp:
                        if 'position' in wp['pose']:
                            pos = wp['pose']['position']
                            waypoint_list.append([pos['x'], pos['y']])
                        else:
                            waypoint_list.append([wp['pose']['x'], wp['pose']['y']])
                    elif 'position' in wp:
                        pos = wp['position']
                        waypoint_list.append([pos['x'], pos['y']])
                elif isinstance(wp, (list, tuple)) and len(wp) >= 2:
                    # Handle list format: [x, y] or [x, y, z]
                    waypoint_list.append([wp[0], wp[1]])
            
            self.waypoints = np.array(waypoint_list)
            
            if len(self.waypoints) == 0:
                raise ValueError("No valid waypoints found in YAML file")
            
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {yaml_path}')
            self.get_logger().info(f'First waypoint: {self.waypoints[0]}')
            self.get_logger().info(f'Last waypoint: {self.waypoints[-1]}')
            
        except FileNotFoundError:
            self.get_logger().error(f'Waypoint file not found: {self.waypoint_file}')
            self.use_default_waypoints()
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints from YAML: {e}')
            self.use_default_waypoints()
    
    def use_default_waypoints(self):
        """Use default waypoints as fallback"""
        self.waypoints = np.array([
            [0.0, 0.0],
            [5.0, 0.0],
            [10.0, 2.0],
            [15.0, 2.0],
            [20.0, 0.0]
        ])
        self.get_logger().warn('Using default waypoints')
    
    def publish_path_visualization(self):
        if len(self.waypoints) == 0:
            return
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(wp[0])
            pose.pose.position.y = float(wp[1])
            if len(wp) >= 3:
                q = self.yaw_to_quaternion(wp[2])
                pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
            else:
                pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_viz_pub.publish(path_msg)

    def publish_target_marker(self, point):
        if point is None:
            return
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0.4
        marker.scale.x = marker.scale.y = marker.scale.z = 0.4
        marker.color.r = 1.0
        marker.color.a = 1.0
        self.target_marker_pub.publish(marker)

    def imu_callback(self, msg):
        """Process IMU data to get current orientation"""
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        # self.current_yaw = yaw
        
        # Extract velocity from angular velocity (approximate)
        # Note: For better velocity estimation, consider using wheel encoders or GPS
        if hasattr(msg, 'angular_velocity'):
            self.current_velocity = abs(msg.angular_velocity.z) * self.wheelbase
    
    def pose_callback(self, msg):
        """Process pose data from lidar localization"""
        self.current_pose = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])
        
        # Also extract yaw from pose if IMU data is not available
        # if self.current_yaw is None:
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw

        # t = TransformStamped()

        # t.header.stamp = self.get_clock().now().to_msg()
        # t.header.frame_id = 'map'
        # t.child_frame_id = 'carver'
        # t.transform.translation.x = msg.pose.pose.position.x
        # t.transform.translation.y = msg.pose.pose.position.y
        # t.transform.translation.z = 0.0
        # t.transform.rotation.x = orientation_q.x
        # t.transform.rotation.y = orientation_q.y
        # t.transform.rotation.z = orientation_q.z
        # t.transform.rotation.w = orientation_q.w

        # self.tf_broadcaster.sendTransform(t)
    
    def calculate_dynamic_lookahead(self):
        """Calculate dynamic look-ahead distance based on current velocity"""
        lookahead = self.min_lookahead + self.lookahead_gain * self.current_velocity
        return np.clip(lookahead, self.min_lookahead, self.max_lookahead)
    
    def find_target_point(self, lookahead_distance):
        """Robust pure-pursuit target selection with proper nearest-point search."""

        if self.current_pose is None or len(self.waypoints) < 2:
            return None

        # --- 1. Find nearest waypoint to robot ---
        dists = np.linalg.norm(self.waypoints - self.current_pose, axis=1)
        nearest_idx = np.argmin(dists)

        # Ensure monotonic forward movement
        if nearest_idx < self.current_waypoint_idx:
            nearest_idx = self.current_waypoint_idx

        self.current_waypoint_idx = nearest_idx

        # --- 2. Walk forward until we reach lookahead distance ---
        L = lookahead_distance
        accum_dist = 0.0

        for i in range(nearest_idx, len(self.waypoints) - 1):
            p1 = self.waypoints[i]
            p2 = self.waypoints[i + 1]
            segment = p2 - p1
            seg_len = np.linalg.norm(segment)

            if seg_len < 1e-6:
                continue

            if accum_dist + seg_len >= L:
                # Interpolate inside this segment
                t = (L - accum_dist) / seg_len
                target = p1 + t * segment
                return target

            accum_dist += seg_len

        # Fallback to last waypoint
        return self.waypoints[-1]

   
    def calculate_steering_angle(self, target_point):
        """Calculate steering angle using pure pursuit algorithm"""
        if target_point is None or self.current_pose is None or self.current_yaw is None:
            return 0.0
        
        # Transform target point to vehicle frame
        dx = target_point[0] - self.current_pose[0]
        dy = target_point[1] - self.current_pose[1]
        
        # Rotate to vehicle frame
        alpha = math.atan2(dy, dx) - self.current_yaw
        
        # Calculate look-ahead distance
        ld = np.linalg.norm(target_point - self.current_pose)
        
        if ld < 0.1:  # Avoid division by zero
            return 0.0
        
        # Pure pursuit formula
        steering_angle = math.atan2(2.0 * self.wheelbase * math.sin(alpha), ld)
        
        # Clamp steering angle
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)
        
        return steering_angle
    
    def calculate_speed(self, steering_angle):
        """Calculate target speed based on steering angle (slow down in turns)"""
        # Reduce speed proportionally to steering angle
        speed_factor = 1.0 - abs(steering_angle) / self.max_steering_angle * 0.5
        self.current_velocity = self.target_speed * speed_factor
        return np.clip(self.current_velocity, 0.0, self.max_speed)
    
    def control_loop(self):
        """Main control loop"""
        if self.current_pose is None or self.current_yaw is None:
            self.get_logger().warn('Waiting for pose and IMU data...', throttle_duration_sec=2.0)
            return
        
        # if not hasattr(self, 'controller_enabled') or not self.controller_enabled:
        #     return
        
        if len(self.waypoints) == 0:
            self.get_logger().warn('No waypoints loaded', throttle_duration_sec=2.0)
            return
        
        # Check if we've reached the final waypoint
        # final_dist = np.linalg.norm(self.current_pose - self.waypoints[-1])
        # if final_dist < 0.5:  # Within 0.5m of final waypoint
        #     self.get_logger().info('Reached final waypoint, stopping')
        #     self.speed_pub.publish(Float32(data=0.0))
        #     self.steering_pub.publish(Float32(data=0.0))
        #     return
        
        # Calculate dynamic look-ahead distance
        lookahead_distance = self.calculate_dynamic_lookahead()
        
        # Find target point
        target_point = self.find_target_point(lookahead_distance)
        
        if target_point is None:
            self.get_logger().warn('No target point found')
            return
        
        final_dist = np.linalg.norm(self.current_pose - self.waypoints[-1, :2])
        if final_dist < 1.0:
            self.speed_pub.publish(Float32(data=0.0))
            self.steering_pub.publish(Float32(data=0.0))
            self.get_logger().info('Reached final waypoint, stopping')
            # Optionally disable controller here
            self.turnoff = True
            return
            
        # Calculate steering angle
        steering_angle = self.calculate_steering_angle(target_point)
        
        # Calculate target speed
        speed = self.calculate_speed(steering_angle)
        
        # Publish Ackermann command
        if not self.turnoff:
            self.speed_pub.publish(Float32(data=speed))
            self.steering_pub.publish(Float32(data=steering_angle))
        
        # Debug logging
            self.get_logger().info(
                f'Lookahead: {lookahead_distance:.2f}m, '
                f'Steering: {math.degrees(steering_angle):.2f}°, '
                f'Speed: {speed:.2f}m/s, '
                f'Target Point: ({target_point[0]:.2f}, {target_point[1]:.2f}), '
                f'Waypoint: {self.current_waypoint_idx}/{len(self.waypoints)}'
                f'Pos: {self.current_pose}'
            )


def main(args=None):
    rclpy.init(args=args)
    controller = PurePursuitController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()