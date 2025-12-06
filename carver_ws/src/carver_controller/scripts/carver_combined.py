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

# --- CONSTANTS FROM ARTICLE [cite: 131] ---
# Note: k_crosstrack (k in article) and k_l (k1 in article) should be optimized
# Optimized values from Fig. 4/5 results (k_l=0.4, k=1.9) are used as defaults.
# However, for demonstration, k_l=0.4 and k_crosstrack=1.9 are used based on optimization results [cite: 165]
PATH_POINT_DISTANCE_D = 0.5  # d [m] [cite: 131]
MIN_TURNING_RADIUS_R_MIN = 7.0 # R_min [m] [cite: 131]
K_MIN = 0.2                  # k_min [-] [cite: 131]
K_MAX = 0.8                  # k_max [-] [cite: 131]
WHEELBASE_W = 0.8            # Assuming a standard vehicle wheelbase for formula [cite: 73]


class CombinedController(Node):
    """
    ROS 2 Node implementing the Combined Path Following Controller (Stanley + Pure Pursuit)
    with dynamic weighting based on path curvature, as described in the research article.
    """
    def __init__(self):
        super().__init__("combined_controller_node")

        # --- Publishers & Subscribers/Services (Standard ROS Setup) ---
        self.steering_pub = self.create_publisher(Float32, "/steering_angle", 10)
        self.speed_pub = self.create_publisher(Float32, "/controller_speed", 10)
        self.path_viz_pub = self.create_publisher(Path, "/path_visualization", 10)
        self.target_marker_pub = self.create_publisher(Marker, "/target_point", 10)
        self.create_subscription(Odometry, "/state_estimator/pose", self.pose_callback, 10)
        self.create_service(SetBool, "/auto/enable", self.bool_service_callback)

        # --- Parameters (Mixing Stanley and Pure Pursuit required params) ---
        self.declare_parameter("k_crosstrack", 0.05)      # Stanley Gain k [cite: 165, 138]
        # self.declare_parameter("k_heading", 0.0)         # Heading Error is not explicitly used in Stanley formula, but set to 0.0 here.
        self.declare_parameter("ks_gain", 0.1)           # Stanley Softening Gain
        self.declare_parameter("target_speed", 2.0)
        self.declare_parameter("max_steer", 0.6)
        self.declare_parameter("steering_sign", 1.0)
        self.declare_parameter("lookahead_gain_k1", 0.1) # Pure Pursuit Gain k_l [cite: 165, 138, 120]
        self.declare_parameter(
            "path_file",
            os.path.expanduser("~/carver_ws/src/carver_controller/path/trajectory1000.yaml"),
        )
        
        # --- Parameter Retrieval ---
        self.k_crosstrack = self.get_parameter("k_crosstrack").value
        # self.k_heading = self.get_parameter("k_heading").value # Unused in this formulation but kept for structure
        self.ks = self.get_parameter("ks_gain").value
        self.target_speed = self.get_parameter("target_speed").value
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
        self.controller_enabled = True  # START ENABLED BY DEFAULT
        self.turnoff = False
        self.heading_initialized = False # Needed for velocity calculation if used

        # --- Load Path & Pre-calculate Constants ---
        self.waypoints = self.load_path_from_yaml(path_file)
        
        # Pre-calculate gamma_norm [cite: 103, 106]
        # Formula (7): Y_norm = 2 * arcsin(d / (2 * R_min))
        arg = PATH_POINT_DISTANCE_D / (2 * MIN_TURNING_RADIUS_R_MIN)
        arg = np.clip(arg, -1.0, 1.0)
        self.gamma_norm = 2 * math.asin(arg) # in Radians (The table lists 4.1 deg [cite: 131])

        self.get_logger().info(f"Combined Controller initialized. Gamma_norm: {math.degrees(self.gamma_norm):.2f}°")
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from path file")
        self.get_logger().info("Controller ENABLED by default (use /auto/enable service to disable)")
        
        # --- Timers ---
        self.create_timer(0.02, self.control_loop) # 50 Hz
        self.create_timer(1.0, self.publish_path_visualization)

    # --- Standard ROS/Utility Methods (Loading, Callbacks, Normalization) ---

    def load_path_from_yaml(self, filepath):
        # (Path loading logic remains identical to previous examples)
        # Simplified for brevity in this final combined code block
        try:
            # ... (Path loading logic from Stanley code) ...
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
            return waypoints

        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
            return self.use_default_waypoints()

    def use_default_waypoints(self):
        # (Default waypoint logic remains identical)
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
        
        self.get_logger().debug(
            f"Pose Update -> X: {self.current_x:.2f}, Y: {self.current_y:.2f}, "
            f"Yaw: {math.degrees(self.current_yaw):.2f}°, Speed: {self.current_speed:.2f} m/s",
            throttle_duration_sec=1.0
        )

    def bool_service_callback(self, request, response):
        self.controller_enabled = request.data
        if not self.controller_enabled:
            self.publish_commands(0.0, 0.0)
            self.current_waypoint_idx = 0
            self.get_logger().info("Controller DISABLED - stopping vehicle")
        else:
            self.get_logger().info("Controller ENABLED - starting path following")
        response.success = True
        return response

    def normalize_angle(self, angle):
        # (Normalize angle utility)
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    # --- Core Pure Pursuit & Stanley Utility Methods ---

    def find_target_point_and_segment(self, L):
        """Finds target point for PP, and nearest segment for Stanley/CTE."""
        if self.current_pose is None or len(self.waypoints) < 2:
            return None, None
        
        # 1. Find nearest waypoint and ensure monotonic movement
        dists = np.linalg.norm(self.waypoints[:, :2] - self.current_pose, axis=1)
        nearest_idx = np.argmin(dists)
        if nearest_idx < self.current_waypoint_idx:
            nearest_idx = self.current_waypoint_idx
        self.current_waypoint_idx = nearest_idx

        # 2. Walk forward until we reach lookahead distance (L)
        accum_dist = 0.0
        target_point = None
        target_segment_idx = nearest_idx

        for i in range(nearest_idx, len(self.waypoints) - 1):
            p1 = self.waypoints[i, :2]
            p2 = self.waypoints[i + 1, :2]
            segment = p2 - p1
            seg_len = np.linalg.norm(segment)
            if seg_len < 1e-6: continue

            if accum_dist + seg_len >= L:
                # Interpolate for target point (required for PP and angle Gamma)
                t = (L - accum_dist) / seg_len
                target_point = p1 + t * segment
                target_segment_idx = i
                break
            accum_dist += seg_len

        # Fallback to last waypoint
        if target_point is None:
            target_point = self.waypoints[-1, :2]
            target_segment_idx = len(self.waypoints) - 2
        
        return target_point, nearest_idx # Return target point and nearest index (for CTE)

    def calculate_pure_pursuit_steer(self, target_point):
        """Calculates Pure Pursuit steering angle (delta_PP)[cite: 71]."""
        if target_point is None: return 0.0
        
        # 1. Calculate Lookahead Distance L (dynamic L used for target selection)
        ld = np.linalg.norm(target_point - self.current_pose)
        if ld < 0.1:
            self.get_logger().debug("  Pure Pursuit: Lookahead distance too small", throttle_duration_sec=0.5)
            return 0.0
        
        self.get_logger().debug(f"  Pure Pursuit: Actual lookahead distance = {ld:.2f} m", throttle_duration_sec=0.5)
        
        # 2. Calculate angle alpha
        dx = target_point[0] - self.current_pose[0]
        dy = target_point[1] - self.current_pose[1]
        alpha = math.atan2(dy, dx) - self.current_yaw
        alpha = self.normalize_angle(alpha)
        
        self.get_logger().debug(f"  Pure Pursuit: α = {math.degrees(alpha):.2f}°", throttle_duration_sec=0.5)
        
        # 3. Pure Pursuit Formula (5): delta = tan-1(2W*sin(alpha)/L) [cite: 71]
        steer_pp = math.atan2(2.0 * WHEELBASE_W * math.sin(alpha), ld)
        return steer_pp

    def calculate_stanley_steer(self, nearest_segment_idx):
        """Calculates Stanley steering angle (delta_st)[cite: 88]."""
        # 1. Calculate CTE
        cte = self.calculate_cross_track_error(nearest_segment_idx)
        self.get_logger().debug(f"  Stanley: CTE = {cte:.3f} m", throttle_duration_sec=0.5)
        
        # 2. Calculate Heading Error (Theta_e) [cite: 81]
        # Stanley original: Theta_e = Path Tangent - Vehicle Heading
        # For simplicity and adherence to original Stanley, we use the path segment angle.
        
        if nearest_segment_idx >= len(self.waypoints) - 1: return 0.0
        
        p1 = self.waypoints[nearest_segment_idx, :2]
        p2 = self.waypoints[nearest_segment_idx + 1, :2]
        
        # Path Tangent Angle (Yaw of the path segment)
        path_heading = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        
        # Heading Error (Theta_e)
        theta_e = self.normalize_angle(path_heading - self.current_yaw)
        self.get_logger().debug(
            f"  Stanley: Path heading = {math.degrees(path_heading):.2f}°, "
            f"Vehicle yaw = {math.degrees(self.current_yaw):.2f}°, θ_e = {math.degrees(theta_e):.2f}°",
            throttle_duration_sec=0.5
        )
        
        # 3. Stanley Formula (6): delta = tan-1(-ke/v_f) - Theta_e [cite: 88]
        # Note: Sign convention is adjusted to match the typical formula $\delta = \theta_e + \arctan(k \cdot e / v)$ with corrected signs.
        speed = max(self.current_speed, 0.3)
        
        # Cross-track term (using -1 * atan2 to handle signs correctly)
        crosstrack_term = -1 * math.atan2(self.k_crosstrack * cte, self.ks + speed)
        self.get_logger().debug(f"  Stanley: Crosstrack term = {math.degrees(crosstrack_term):.2f}°", throttle_duration_sec=0.5)
        
        # The Stanley formula in the article is delta = tan-1(-ke/v_f) - Theta_e [cite: 88]
        # To match the familiar delta = Theta_e + arctan(k*e/v) but with correct signs:
        steer_st = theta_e + crosstrack_term
        return steer_st

    def calculate_cross_track_error(self, segment_idx):
        """Calculates signed CTE (Identical to verified Stanley code)."""
        if self.current_pose is None or segment_idx >= len(self.waypoints) - 1:
            return 0.0

        a = self.waypoints[segment_idx, :2]
        b = self.waypoints[segment_idx + 1, :2]
        p = self.current_pose

        ab = b - a
        ap = p - a

        # Vector Projection: proj = (ap · ab) / (ab · ab)
        ab_dot_ab = np.dot(ab, ab)
        if ab_dot_ab < 1e-6: return np.linalg.norm(ap)

        proj = np.dot(ap, ab) / ab_dot_ab
        proj = np.clip(proj, 0.0, 1.0)

        # Find closest point on path
        closest = a + proj * ab

        # Vector from closest point to robot
        error = p - closest

        # Signed CTE: cross = (ab x error) / |ab|
        cross = ab[0] * error[1] - ab[1] * error[0]
        cte = cross / np.linalg.norm(ab)

        return cte

    def calculate_path_angle_gamma(self, target_segment_idx):
        """
        Calculates the current angle gamma (γ) between path sections 
        around the lookahead target point, based on equation (8) and Fig. 3[cite: 109, 115].
        """
        wps = self.waypoints[:, :2]
        
        # Target Point is interpolated in find_target_point. Here we approximate by using 
        # the path segment that contains the target point.
        
        # Ensure we have points for i-1, i, and i+1
        if target_segment_idx < 1 or target_segment_idx >= len(wps) - 2:
            return 0.0
            
        # P_before = wps[i-1], P_target = wps[i], P_after = wps[i+1]
        # Note: The article calculates gamma using target point +/- 1. Here we use segment indices.
        
        # Vector 1: Section before target segment
        p_prev = wps[target_segment_idx]
        p_current = wps[target_segment_idx + 1]
        
        # Vector 2: Section after target segment
        p_next = wps[target_segment_idx + 2]
        
        # Angle of the section BEFORE the target segment (Section i to i+1)
        angle1 = math.atan2(p_current[1] - p_prev[1], p_current[0] - p_prev[0])
        
        # Angle of the section AFTER the target segment (Section i+1 to i+2)
        angle2 = math.atan2(p_next[1] - p_current[1], p_next[0] - p_current[0])

        # Angle gamma is the difference between the two section angles [cite: 109]
        gamma = self.normalize_angle(angle2 - angle1)
        return abs(gamma)

    # --- Main Combined Control Logic ---

    def calculate_combined_steer(self):
        # 1. Dynamic Lookahead Distance [cite: 120]
        # Formula (9): L = 2 + k_l * v
        L = 1.5 + self.lookahead_gain_k1 * self.current_speed
        self.get_logger().info(f"Step 1: Lookahead L = {L:.2f} m (speed: {self.current_speed:.2f} m/s)", throttle_duration_sec=0.5)
        
        # 2. Find Target Point (for PP) and Nearest Segment (for Stanley/Weighting)
        target_point, nearest_segment_idx = self.find_target_point_and_segment(L)
        if target_point is None:
            self.get_logger().warn("Step 2: No target point found!", throttle_duration_sec=0.5)
            return 0.0
        
        self.get_logger().info(
            f"Step 2: Target point = [{target_point[0]:.2f}, {target_point[1]:.2f}], "
            f"Nearest segment idx = {nearest_segment_idx}",
            throttle_duration_sec=0.5
        )
        
        # 3. Calculate Individual Steering Commands
        delta_pp = self.calculate_pure_pursuit_steer(target_point)
        self.get_logger().info(f"Step 3a: Pure Pursuit δ_PP = {math.degrees(delta_pp):.2f}°", throttle_duration_sec=0.5)
        
        delta_st = self.calculate_stanley_steer(nearest_segment_idx)
        self.get_logger().info(f"Step 3b: Stanley δ_ST = {math.degrees(delta_st):.2f}°", throttle_duration_sec=0.5)
        
        # 4. Calculate Dynamic Weighting based on Path Curvature (Gamma)
        
        # Calculate current path curvature angle (gamma)
        gamma = self.calculate_path_angle_gamma(nearest_segment_idx)
        self.get_logger().info(f"Step 4a: Gamma = {math.degrees(gamma):.2f}°", throttle_duration_sec=0.5)
        
        # Normalize gamma (γ / γ_norm) [cite: 123]
        gamma_normalized = gamma / self.gamma_norm if self.gamma_norm > 1e-6 else 0.0
        gamma_normalized = np.clip(gamma_normalized, 0.0, 1.0) # Clip to 1.0 for k_max [cite: 128]
        self.get_logger().info(f"Step 4b: Gamma normalized = {gamma_normalized:.3f}", throttle_duration_sec=0.5)

        # Weight factor of Pure Pursuit (k_pp) [cite: 123]
        # Formula (10): k_pp = k_min + (gamma / gamma_norm) * (k_max - k_min)
        k_pp = K_MIN + gamma_normalized * (K_MAX - K_MIN)
        
        # Weight factor of Stanley (k_st) [cite: 124]
        # Formula (11): k_st = 1 - k_pp
        k_st = 1.0 - k_pp
        
        self.get_logger().info(f"Step 5: Weights -> k_PP = {k_pp:.3f}, k_ST = {k_st:.3f}", throttle_duration_sec=0.5)
        
        # 5. Combine Steering Commands 
        # Formula (12): delta = k_pp * delta_pp + k_st * delta_st
        steer = k_pp * delta_pp + k_st * delta_st
        
        self.get_logger().info(
            f"Step 6: Combined δ = {k_pp:.2f}*{math.degrees(delta_pp):.2f}° + {k_st:.2f}*{math.degrees(delta_st):.2f}° = {math.degrees(steer):.2f}°",
            throttle_duration_sec=0.5
        )
        
        # Clamp steering angle
        steer_before_clamp = steer
        steer = np.clip(steer, -self.max_steer, self.max_steer)
        
        if abs(steer_before_clamp - steer) > 0.001:
            self.get_logger().info(
                f"Step 7: Clamped {math.degrees(steer_before_clamp):.2f}° -> {math.degrees(steer):.2f}°",
                throttle_duration_sec=0.5
            )

        self.get_logger().info(
            f"FINAL: k_pp: {k_pp:.2f} | k_st: {k_st:.2f} | Gamma: {math.degrees(gamma):.2f}° "
            f"| δ_PP: {math.degrees(delta_pp):.2f}° | δ_ST: {math.degrees(delta_st):.2f}° "
            f"| δ_Final: {self.steering_sign * math.degrees(steer):.2f}°",
            throttle_duration_sec=0.5,
        )
        return steer

    def control_loop(self):
        # STEP 1: Check if pose data has been received first
        if self.current_pose is None:
            self.get_logger().warn("Waiting for pose data from /state_estimator/pose", throttle_duration_sec=2.0)
            return
        
        # STEP 2: Check if controller is enabled and waypoints are loaded
        if not self.controller_enabled:
            self.get_logger().debug("Controller disabled (call service to enable)", throttle_duration_sec=2.0)
            return
            
        if len(self.waypoints) == 0:
            self.get_logger().warn("No waypoints loaded!", throttle_duration_sec=2.0)
            return
        
        self.get_logger().info("=== Control Loop Active ===", throttle_duration_sec=1.0)
        
        # STEP 3: Check if already turned off
        if self.turnoff:
            self.get_logger().debug("Controller turned off (reached goal)", throttle_duration_sec=2.0)
            return
        
        # STEP 4: Check if reached final waypoint
        final_dist = np.linalg.norm(self.current_pose - self.waypoints[-1, :2])
        self.get_logger().info(f"Distance to goal: {final_dist:.2f} m", throttle_duration_sec=0.5)
        
        if final_dist < 0.5:
            self.publish_commands(0.0, 0.0)
            self.get_logger().info('✓ Reached final waypoint, stopping')
            self.turnoff = True
            return

        # STEP 5: Calculate and publish steering
        self.get_logger().info("Calculating steering command...", throttle_duration_sec=0.5)
        steer = self.calculate_combined_steer()
        self.publish_commands(steer, self.target_speed)

    def publish_commands(self, steering_angle, speed):
        # (Command publishing logic)
        published_steer = self.steering_sign * float(steering_angle)
        published_speed = float(speed)
        
        self.steering_pub.publish(Float32(data=published_steer))
        self.speed_pub.publish(Float32(data=published_speed))
        
        # Log published values
        self.get_logger().info(
            f"Published -> Steering: {math.degrees(published_steer):.2f}° | Speed: {published_speed:.2f} m/s",
            throttle_duration_sec=0.5
        )
        
        # Visualization markers (optional, kept for debugging)
        # target, _ = self.find_target_point_and_segment(2.0 + self.lookahead_gain_k1 * self.current_speed)
        # self.publish_target_marker(target)
        
    def publish_target_marker(self, point):
        # (Marker publishing logic)
        if point is None: return
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target_point"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.scale.x = marker.scale.y = marker.scale.z = 0.4
        marker.color.r = 1.0; marker.color.a = 1.0
        self.target_marker_pub.publish(marker)

# --- Main ROS 2 Execution ---

def main(args=None):
    rclpy.init(args=args)
    node = CombinedController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()