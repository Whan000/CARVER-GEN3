#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import math
import numpy as np
import yaml
import os
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


class Stanley(Node):
    def __init__(self):
        super().__init__("stanley_node")

        self.steering_pub = self.create_publisher(Float32, "/steering_angle", 10)
        self.speed_pub = self.create_publisher(Float32, "/controller_speed", 10)
        self.path_viz_pub = self.create_publisher(Path, "/path_visualization", 10)
        self.target_marker_pub = self.create_publisher(Marker, "/target_point", 10)

        self.create_subscription(
            Odometry, "/state_estimator/pose", self.pose_callback, 10
        )
        self.create_service(SetBool, "/auto/enable", self.bool_service_callback)

        self.declare_parameter("k_heading", 0.2) #0.1
        self.declare_parameter("k_crosstrack", 0.1) #0.15
        self.declare_parameter("ks_gain", 0.5)
        self.declare_parameter("target_speed", 1.00)
        self.declare_parameter("max_steer", 0.6)
        self.declare_parameter("steering_sign", 1.0)
        self.declare_parameter("L_front_axle", 0.5)  # Front axle offset
        self.declare_parameter(
            "path_file",
            "/home/katana/Desktop/array/carver_ws/src/carver_controller/path/trajectory1000.yaml",
        )

        self.k_heading = self.get_parameter("k_heading").value
        self.k_crosstrack = self.get_parameter("k_crosstrack").value
        self.ks = self.get_parameter("ks_gain").value
        self.target_speed = self.get_parameter("target_speed").value
        self.max_steer = self.get_parameter("max_steer").value
        self.steering_sign = self.get_parameter("steering_sign").value
        self.L_front_axle = self.get_parameter("L_front_axle").value
        self.turnoff = False

        path_file = self.get_parameter("path_file").value

        # Current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_speed = 0.0
        self.current_pose = None  # Robot position [x, y]

        # Waypoint tracking
        self.current_waypoint_idx = 0

        self.controller_enabled = False

        self.waypoints = self.load_path_from_yaml(path_file)
        if len(self.waypoints) > 0:
            self.publish_path_visualization()

        self.create_timer(0.02, self.control_loop)  # 50 Hz
        self.create_timer(1.0, self.publish_path_visualization)

        self.get_logger().info(
            f"Stanley ready | {len(self.waypoints)} waypoints loaded"
        )

    def load_path_from_yaml(self, filepath):
        """Load waypoints from YAML file with robust format handling"""
        try:
            yaml_path = os.path.expanduser(filepath)

            with open(yaml_path, "r") as file:
                data = yaml.safe_load(file)

            if "waypoints" in data:
                waypoints_data = data["waypoints"]
            elif "path" in data:
                waypoints_data = data["path"]
            elif "poses" in data:
                waypoints_data = data["poses"]
            else:
                waypoints_data = data

            waypoint_list = []
            for wp in waypoints_data:
                if isinstance(wp, dict):
                    if "x" in wp and "y" in wp:
                        yaw = wp.get("yaw", 0.0)
                        waypoint_list.append([wp["x"], wp["y"], yaw])
                    elif "pose" in wp:
                        if "position" in wp["pose"]:
                            pos = wp["pose"]["position"]
                            yaw = wp.get("yaw", 0.0)
                            waypoint_list.append([pos["x"], pos["y"], yaw])
                        else:
                            yaw = wp["pose"].get("yaw", 0.0)
                            waypoint_list.append(
                                [wp["pose"]["x"], wp["pose"]["y"], yaw]
                            )
                    elif "position" in wp:
                        pos = wp["position"]
                        yaw = wp.get("yaw", 0.0)
                        waypoint_list.append([pos["x"], pos["y"], yaw])
                elif isinstance(wp, (list, tuple)) and len(wp) >= 2:
                    if len(wp) >= 3:
                        waypoint_list.append([wp[0], wp[1], wp[2]])
                    else:
                        waypoint_list.append([wp[0], wp[1], 0.0])

            waypoints = np.array(waypoint_list)

            if len(waypoints) == 0:
                raise ValueError("No valid waypoints found in YAML file")

            self.get_logger().info(
                f"Loaded {len(waypoints)} waypoints from {yaml_path}"
            )
            self.get_logger().info(f"First waypoint: {waypoints[0]}")
            self.get_logger().info(f"Last waypoint: {waypoints[-1]}")

            return waypoints

        except FileNotFoundError:
            self.get_logger().error(f"Waypoint file not found: {filepath}")
            return self.use_default_waypoints()
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints from YAML: {e}")
            return self.use_default_waypoints()

    def use_default_waypoints(self):
        """Use default waypoints as fallback"""
        default_wps = np.array(
            [
                [0.0, 0.0, 0.0],
                [5.0, 0.0, 0.0],
                [10.0, 2.0, 0.0],
                [15.0, 2.0, 0.0],
                [20.0, 0.0, 0.0],
            ]
        )
        self.get_logger().warn("Using default waypoints")
        return default_wps

    def yaw_to_quaternion(self, yaw):
        s = math.sin(yaw / 2.0)
        c = math.cos(yaw / 2.0)
        return [0.0, 0.0, s, c]

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
                (
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w,
                ) = q
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

    def pose_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_pose = np.array([self.current_x, self.current_y])

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.hypot(vx, vy)

        q = msg.pose.pose.orientation
        self.current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def bool_service_callback(self, request, response):
        self.controller_enabled = request.data
        response.success = True
        response.message = "Enabled" if self.controller_enabled else "Disabled"
        self.get_logger().info(response.message)
        if not self.controller_enabled:
            self.publish_commands(0.0, 0.0)
            self.current_waypoint_idx = 0  # Reset waypoint index
        return response

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def find_target_point(self):
        """Find closest point on path to vehicle's front axle (Stanley approach)"""
        if self.current_pose is None or len(self.waypoints) < 2:
            return None, None

        # Calculate front axle position (Stanley uses front axle, not rear/center)
        fx = self.current_x + self.L_front_axle * math.cos(self.current_yaw)
        fy = self.current_y + self.L_front_axle * math.sin(self.current_yaw)
        front_axle_pos = np.array([fx, fy])

        # Find closest waypoint to front axle
        dists = np.linalg.norm(self.waypoints[:, :2] - front_axle_pos, axis=1)
        target_idx = np.argmin(dists)

        # Ensure monotonic forward movement
        if target_idx < self.current_waypoint_idx:
            target_idx = self.current_waypoint_idx
        
        self.current_waypoint_idx = target_idx

        # Target point is the closest point on path
        target_point = self.waypoints[target_idx, :2]
        
        # For Stanley, use the same point for both heading and CTE
        return target_point, target_idx

    def calculate_cross_track_error(self, target_idx):
        """Calculate cross-track error using perpendicular distance to vehicle heading"""
        if self.current_pose is None or target_idx >= len(self.waypoints):
            return 0.0

        # Front axle position
        fx = self.current_x + self.L_front_axle * math.cos(self.current_yaw)
        fy = self.current_y + self.L_front_axle * math.sin(self.current_yaw)

        # Error vector from front axle to target point
        target_point = self.waypoints[target_idx, :2]
        error_x = target_point[0] - fx
        error_y = target_point[1] - fy

        # Project error onto perpendicular to vehicle heading
        # Perpendicular vector: [-sin(yaw), cos(yaw)]
        cte = error_x * (-math.sin(self.current_yaw)) + error_y * math.cos(self.current_yaw)

        return cte

    def compute_heading_error(self, target_idx):
        """Compute heading error between vehicle and path tangent"""
        if target_idx >= len(self.waypoints):
            return 0.0
        
        # Path heading at target point
        path_yaw = self.waypoints[target_idx, 2]
        
        # Heading error
        heading_err = self.normalize_angle(path_yaw - self.current_yaw)
        
        return heading_err

    def stanley_control(self):
        target_point, target_idx = self.find_target_point()
        if target_point is None:
            return 0.0

        self.publish_target_marker(target_point)
        
        # Heading error: difference between path tangent and vehicle heading
        heading_err = self.compute_heading_error(target_idx)
        
        # Cross-track error: perpendicular distance from front axle to path
        cte = self.calculate_cross_track_error(target_idx)
        
        # Stanley control law
        speed = max(self.current_speed, 0.3)
        heading_term = heading_err  # No gain needed, it's already the error
        crosstrack_term = math.atan2(self.k_crosstrack * cte, speed)
        
        steer = heading_term + crosstrack_term
        steer = np.clip(steer, -self.max_steer, self.max_steer)

        self.get_logger().info(
            f"WP_idx: {target_idx}  CTE: {cte:+.3f}m  HE: {math.degrees(heading_err):+6.1f}°  "
            f"Steer: {self.steering_sign*math.degrees(steer):+6.1f}°",
            throttle_duration_sec=0.5,
        )
        return steer

    def control_loop(self):
        # if not self.controller_enabled:
        #     return
        
        final_dist = np.linalg.norm(self.current_pose - self.waypoints[-1, :2])
        if final_dist < 1.0:
            self.publish_commands(0.0, 0.0)
            self.get_logger().info('Reached final waypoint, stopping')
            # Optionally disable controller here
            self.turnoff = True
            return
        
        if not self.turnoff:
            steer = self.stanley_control()
            self.publish_commands(steer, self.target_speed)

    def publish_commands(self, steering_angle, speed):
        self.steering_pub.publish(
            Float32(data=self.steering_sign * float(steering_angle))
        )
        self.speed_pub.publish(Float32(data=float(speed)))


def main(args=None):
    rclpy.init(args=args)
    node = Stanley()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()