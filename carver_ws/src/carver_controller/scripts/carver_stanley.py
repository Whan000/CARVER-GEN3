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
        self.speed_pub = self.create_publisher(Float32, "/target_speed", 10)
        self.path_viz_pub = self.create_publisher(Path, "/path_visualization", 10)
        self.target_marker_pub = self.create_publisher(Marker, "/target_point", 10)

        self.create_subscription(
            Odometry, "/state_estimator/pose", self.pose_callback, 10
        )
        self.create_service(SetBool, "/auto/enable", self.bool_service_callback)

        self.declare_parameter("k_heading", 0.25)
        self.declare_parameter("k_crosstrack", 0.1)
        self.declare_parameter("ks_gain", 0.1)
        self.declare_parameter("target_speed", 1.25)
        self.declare_parameter("max_steer", 0.6)
        self.declare_parameter("steering_sign", 1.0)
        self.declare_parameter("lookahead_distance", 5.0)
        self.declare_parameter(
            "path_file",
            "/home/katana/Desktop/array/carver_ws/src/carver_controller/path/trajectory1000.yaml",
        )

        self.k_heading = self.get_parameter("k_heading").value
        self.k_crosstrack = self.get_parameter("k_crosstrack").value
        self.ks = self.get_parameter("ks_gain").value
        self.target_speed = min(self.get_parameter("target_speed").value, 1.5)
        self.max_steer = self.get_parameter("max_steer").value
        self.steering_sign = self.get_parameter("steering_sign").value
        self.lookahead_distance = self.get_parameter("lookahead_distance").value

        path_file = self.get_parameter("path_file").value

        # Current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_speed = 0.0
        self.current_pose = None  # Robot position [x, y]

        # Previous position for velocity vector
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.heading_initialized = False

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
            self.heading_initialized = False
            self.current_waypoint_idx = 0  # Reset waypoint index
        return response

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def find_target_point(self):
        if self.current_pose is None or len(self.waypoints) < 2:
            return None, None

        # --- 1. Find nearest waypoint to robot ---
        dists = np.linalg.norm(self.waypoints[:, :2] - self.current_pose, axis=1)
        nearest_idx = np.argmin(dists)

        # Ensure monotonic forward movement
        if nearest_idx < self.current_waypoint_idx:    
            nearest_idx = self.current_waypoint_idx

        self.current_waypoint_idx = nearest_idx

        # --- 2. Walk forward until we reach lookahead distance ---
        L = self.lookahead_distance
        accum_dist = 0.0
        target_point = None
        target_segment_idx = nearest_idx

        for i in range(nearest_idx, len(self.waypoints) - 1):
            p1 = self.waypoints[i, :2]
            p2 = self.waypoints[i + 1, :2]
            segment = p2 - p1
            seg_len = np.linalg.norm(segment)

            if seg_len < 1e-6:
                continue

            if accum_dist + seg_len >= L:
                # Interpolate inside this segment
                t = (L - accum_dist) / seg_len
                target_point = p1 + t * segment
                target_segment_idx = i
                break

            accum_dist += seg_len

        # Fallback to last waypoint if lookahead extends beyond path
        if target_point is None:
            target_point = self.waypoints[-1, :2]
            target_segment_idx = len(self.waypoints) - 2

        # Return target point for heading error and nearest segment for CTE
        return target_point, nearest_idx

    def calculate_cross_track_error(self, segment_idx):
        """Calculate cross-track error from current position to path segment
        
        Args:
            segment_idx: Index of path segment to calculate CTE from
        """
        
        if self.current_pose is None or segment_idx >= len(self.waypoints) - 1:
            return 0.0

        a = self.waypoints[segment_idx, :2]
        b = self.waypoints[segment_idx + 1, :2]
        p = self.current_pose

        ab = b - a
        ap = p - a

        # Project robot onto path segment
        proj = np.dot(ap, ab) / np.dot(ab, ab)
        proj = np.clip(proj, 0.0, 1.0)

        # Find closest point on path
        closest = a + proj * ab

        # Vector from closest point to robot
        error = p - closest

        # Signed cross-track error (corrected - divide by |ab|)
        cross = ab[0] * error[1] - ab[1] * error[0]
        cte = cross / np.linalg.norm(ab)

        return cte

    def compute_heading_error_from_vectors(self, target_point):
        if target_point is None:
            return 0.0
        if not self.heading_initialized:
            self.prev_x = self.current_x
            self.prev_y = self.current_y
            self.heading_initialized = True
            return 0.0

        dx_velocity = self.current_x - self.prev_x
        dy_velocity = self.current_y - self.prev_y
        ref_vector = np.array([dx_velocity, dy_velocity])

        # Actual Vector = Target (current → target)
        dx_target = target_point[0] - self.current_x
        dy_target = target_point[1] - self.current_y
        actual_vector = np.array([dx_target, dy_target])

        # Calculate norms
        ref_norm = np.linalg.norm(ref_vector)
        actual_norm = np.linalg.norm(actual_vector)

        # Safety: if not moving, use IMU yaw or bearing angle
        if ref_norm < 0.02:  # < 2cm movement
            if actual_norm > 1e-6:
                target_heading = math.atan2(dy_target, dx_target)
                heading_err = self.normalize_angle(target_heading - self.current_yaw)

                self.prev_x = self.current_x
                self.prev_y = self.current_y

                return heading_err
            return 0.0

        if actual_norm < 1e-6:  # Target too close
            self.prev_x = self.current_x
            self.prev_y = self.current_y
            return 0.0
        cos_theta = np.dot(ref_vector, actual_vector) / (ref_norm * actual_norm)
        cos_theta = np.clip(cos_theta, -1.0, 1.0)

        theta_rad = np.arccos(cos_theta)
        cross_product = (
            ref_vector[0] * actual_vector[1] - ref_vector[1] * actual_vector[0]
        )

        if cross_product < 0:
            theta_rad = -theta_rad

        # Debug
        self.get_logger().info(
            f"Vel: [{dx_velocity:.3f}, {dy_velocity:.3f}] "
            f"Tgt: [{dx_target:.3f}, {dy_target:.3f}] "
            f"HE: {math.degrees(theta_rad):.2f}°",
            throttle_duration_sec=1.0,
        )

        # Update previous position
        self.prev_x = self.current_x
        self.prev_y = self.current_y

        return self.normalize_angle(theta_rad)

    def stanley_control(self):
        target, nearest_segment_idx = self.find_target_point()
        if target is None:
            return 0.0

        self.publish_target_marker(target)
        
        # Heading error: toward lookahead target point
        heading_err = self.compute_heading_error_from_vectors(target)
        
        # Cross-track error: from nearest segment on path
        cte = self.calculate_cross_track_error(nearest_segment_idx)
        
        speed = max(self.current_speed, 0.3)
        heading_term = self.k_heading * heading_err
        crosstrack_term = -1*math.atan2(self.k_crosstrack * cte, self.ks + speed)

        steer = heading_term + crosstrack_term
        steer = np.clip(steer, -self.max_steer, self.max_steer)

        self.get_logger().info(
            f"WP_idx: {self.current_waypoint_idx}  CTE: {cte:+.3f}m  HE: {math.degrees(heading_err):+6.1f}°  "
            f"H_term: {math.degrees(heading_term):+6.1f}°  "
            f"CT_term: {math.degrees(crosstrack_term):+6.1f}°  "
            f"Steer: {self.steering_sign*math.degrees(steer):+6.1f}°",
            throttle_duration_sec=0.5,
        )
        return steer

    def control_loop(self):
        if not self.controller_enabled:
            return

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