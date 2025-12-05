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

# --- CONSTANTS FROM ARTICLE ---
PATH_POINT_DISTANCE_D = 0.5  # d [m]
MIN_TURNING_RADIUS_R_MIN = 7.0  # R_min [m]
K_MIN = 0.2  # k_min [-]
K_MAX = 0.8  # k_max [-]
WHEELBASE_W = 0.8  # Wheelbase [m]


class CombinedController(Node):
    """
    Combined Path Following Controller (Stanley + Pure Pursuit)
    with dynamic weighting. Publishes to intermediate topics for multiplexing.
    """
    def __init__(self):
        super().__init__("combined_controller_node")

        # --- Publishers - DIRECTLY to motor topics ---
        self.steering_pub = self.create_publisher(Float32, "/steering_angle", 10)
        self.speed_pub = self.create_publisher(Float32, "/target_speed", 10)
        self.path_viz_pub = self.create_publisher(Path, "/path_visualization", 10)
        self.target_marker_pub = self.create_publisher(Marker, "/target_point", 10)
        
        # --- Subscribers ---
        self.create_subscription(Odometry, "/state_estimator/pose", self.pose_callback, 10)

        # --- Parameters ---
        self.declare_parameter("k_crosstrack", 0.1)
        self.declare_parameter("ks_gain", 0.1)
        self.declare_parameter("target_speed", 1.25)
        self.declare_parameter("max_steer", 0.6)
        self.declare_parameter("steering_sign", 1.0)
        self.declare_parameter("lookahead_gain_k1", 0.1)
        self.declare_parameter(
            "path_file",
            os.path.expanduser("/home/katana/Desktop/array/carver_ws/src/carver_controller/path/trajectory.yaml"),
        )
        
        # --- Parameter Retrieval ---
        self.k_crosstrack = self.get_parameter("k_crosstrack").value
        self.ks = self.get_parameter("ks_gain").value
        self.target_speed = min(self.get_parameter("target_speed").value, 1.5)
        self.max_steer = self.get_parameter("max_steer").value
        self.steering_sign = self.get_parameter("steering_sign").value
        self.lookahead_gain_k1 = self.get_parameter("lookahead_gain_k1").value
        path_file = self.get_parameter("path_file").value

        # --- State Variables ---
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_speed = 0.0
        self.current_pose = None
        self.current_waypoint_idx = 0

        # Controller enabled flag
        self.controller_enabled = False

        # --- Load Path ---
        self.waypoints = self.load_path_from_yaml(path_file)
        
        # Pre-calculate gamma_norm
        arg = PATH_POINT_DISTANCE_D / (2 * MIN_TURNING_RADIUS_R_MIN)
        arg = np.clip(arg, -1.0, 1.0)
        self.gamma_norm = 2 * math.asin(arg)

        # Service for enable/disable
        self.enable_service = self.create_service(
            SetBool, '/auto/enable', self.enable_callback
        )

        self.get_logger().info(f"Combined Controller initialized. Gamma_norm: {math.degrees(self.gamma_norm):.2f}°")
        self.get_logger().info(f"Publishing DIRECTLY to /steering_angle and /target_speed")
        self.get_logger().info(f"Service: /auto/enable (SetBool)")
        self.get_logger().info(f"Controller enabled: {self.controller_enabled}")
        
        # --- Timers ---
        self.create_timer(0.1, self.control_loop)  # 10 Hz
        self.create_timer(1.0, self.publish_path_visualization)

    def enable_callback(self, request, response):
        """Service callback to enable/disable controller"""
        self.controller_enabled = request.data
        
        if self.controller_enabled:
            response.success = True
            response.message = "Combined controller ENABLED"
            self.get_logger().info("Controller ENABLED")
        else:
            response.success = True
            response.message = "Combined controller DISABLED"
            self.get_logger().info("Controller DISABLED")
            # Publish zeros when disabled
            self.steering_pub.publish(Float32(data=0.0))
            self.speed_pub.publish(Float32(data=0.0))
        
        return response

    def load_path_from_yaml(self, filepath):
        """Load waypoints from YAML file."""
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
                    elif "pose" in wp and "position" in wp["pose"]:
                        pos = wp["pose"]["position"]
                        waypoint_list.append([pos["x"], pos["y"], wp["pose"].get("yaw", 0.0)])
                elif isinstance(wp, (list, tuple)) and len(wp) >= 2:
                    waypoint_list.append([wp[0], wp[1], wp[2] if len(wp) >= 3 else 0.0])

            waypoints = np.array(waypoint_list)
            if len(waypoints) == 0:
                raise ValueError("No valid waypoints found in YAML file")
            
            self.get_logger().info(f"Loaded {len(waypoints)} waypoints")
            return waypoints

        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
            return self.use_default_waypoints()

    def use_default_waypoints(self):
        """Provide default waypoints if file loading fails."""
        default_wps = np.array([[0.0, 0.0, 0.0], [5.0, 0.0, 0.0], [10.0, 2.0, 0.0], [20.0, 0.0, 0.0]])
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
    
    def pose_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_pose = np.array([self.current_x, self.current_y])
        
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.hypot(vx, vy)
        
        q = msg.pose.pose.orientation
        self.current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def find_target_point_and_segment(self, L):
        if self.current_pose is None or len(self.waypoints) < 2:
            return None, None
        
        dists = np.linalg.norm(self.waypoints[:, :2] - self.current_pose, axis=1)
        nearest_idx = np.argmin(dists)
        if nearest_idx < self.current_waypoint_idx:
            nearest_idx = self.current_waypoint_idx
        self.current_waypoint_idx = nearest_idx

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
                t = (L - accum_dist) / seg_len
                target_point = p1 + t * segment
                target_segment_idx = i
                break
            accum_dist += seg_len

        if target_point is None:
            target_point = self.waypoints[-1, :2]
            target_segment_idx = len(self.waypoints) - 2
        
        return target_point, nearest_idx

    def calculate_pure_pursuit_steer(self, target_point):
        if target_point is None:
            return 0.0
        
        ld = np.linalg.norm(target_point - self.current_pose)
        if ld < 0.1:
            return 0.0
        
        dx = target_point[0] - self.current_pose[0]
        dy = target_point[1] - self.current_pose[1]
        alpha = math.atan2(dy, dx) - self.current_yaw
        alpha = self.normalize_angle(alpha)
        
        steer_pp = math.atan2(2.0 * WHEELBASE_W * math.sin(alpha), ld)
        return steer_pp

    def calculate_stanley_steer(self, nearest_segment_idx):
        cte = self.calculate_cross_track_error(nearest_segment_idx)
        
        if nearest_segment_idx >= len(self.waypoints) - 1:
            return 0.0
        
        p1 = self.waypoints[nearest_segment_idx, :2]
        p2 = self.waypoints[nearest_segment_idx + 1, :2]
        
        path_heading = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        theta_e = self.normalize_angle(path_heading - self.current_yaw)
        
        speed = max(self.current_speed, 0.3)
        crosstrack_term = -1 * math.atan2(self.k_crosstrack * cte, self.ks + speed)
        
        steer_st = theta_e + crosstrack_term
        return steer_st

    def calculate_cross_track_error(self, segment_idx):
        if self.current_pose is None or segment_idx >= len(self.waypoints) - 1:
            return 0.0

        a = self.waypoints[segment_idx, :2]
        b = self.waypoints[segment_idx + 1, :2]
        p = self.current_pose

        ab = b - a
        ap = p - a

        ab_dot_ab = np.dot(ab, ab)
        if ab_dot_ab < 1e-6:
            return np.linalg.norm(ap)

        proj = np.dot(ap, ab) / ab_dot_ab
        proj = np.clip(proj, 0.0, 1.0)

        closest = a + proj * ab
        error = p - closest

        cross = ab[0] * error[1] - ab[1] * error[0]
        cte = cross / np.linalg.norm(ab)

        return cte

    def calculate_path_angle_gamma(self, target_segment_idx):
        wps = self.waypoints[:, :2]
        
        if target_segment_idx < 1 or target_segment_idx >= len(wps) - 2:
            return 0.0
        
        p_prev = wps[target_segment_idx]
        p_current = wps[target_segment_idx + 1]
        p_next = wps[target_segment_idx + 2]
        
        angle1 = math.atan2(p_current[1] - p_prev[1], p_current[0] - p_prev[0])
        angle2 = math.atan2(p_next[1] - p_current[1], p_next[0] - p_current[0])

        gamma = self.normalize_angle(angle2 - angle1)
        return abs(gamma)

    def calculate_combined_steer(self):
        # Dynamic Lookahead Distance
        L = 1.5 + self.lookahead_gain_k1 * self.current_speed
        
        # Find Target Point and Nearest Segment
        target_point, nearest_segment_idx = self.find_target_point_and_segment(L)
        if target_point is None:
            return 0.0
        
        # Calculate Individual Steering Commands
        delta_pp = self.calculate_pure_pursuit_steer(target_point)
        delta_st = self.calculate_stanley_steer(nearest_segment_idx)
        
        # Calculate Dynamic Weighting
        gamma = self.calculate_path_angle_gamma(nearest_segment_idx)
        gamma_normalized = gamma / self.gamma_norm if self.gamma_norm > 1e-6 else 0.0
        gamma_normalized = np.clip(gamma_normalized, 0.0, 1.0)

        k_pp = K_MIN + gamma_normalized * (K_MAX - K_MIN)
        k_st = 1.0 - k_pp
        
        # Combine Steering Commands
        steer = k_pp * delta_pp + k_st * delta_st
        steer = np.clip(steer, -self.max_steer, self.max_steer)

        self.get_logger().info(
            f"k_pp: {k_pp:.2f} | k_st: {k_st:.2f} | Steer: {math.degrees(steer):.2f}°",
            throttle_duration_sec=1.0,
        )
        return steer

    def control_loop(self):
        """Main control loop at 10 Hz - Only publishes when enabled"""
        # Check if controller is enabled
        if not self.controller_enabled:
            return  # Don't publish anything
        
        # Check if pose initialized
        if self.current_pose is None or len(self.waypoints) == 0:
            return

        # Calculate and publish steering command
        steer = self.calculate_combined_steer()
        self.publish_commands(steer, self.target_speed)

    def publish_commands(self, steering_angle, speed):
        """Publish DIRECTLY to motor topics"""
        self.steering_pub.publish(Float32(data=self.steering_sign * float(steering_angle)))
        self.speed_pub.publish(Float32(data=float(speed)))


def main(args=None):
    rclpy.init(args=args)
    node = CombinedController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()