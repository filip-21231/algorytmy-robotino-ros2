#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, Bool
from rcl_interfaces.msg import SetParametersResult
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger

import math
import numpy as np

class APFNode(Node):

    IR_SENSOR_COUNT = 9
    MIN_IR_DIST = 0.04
    MAX_IR_DIST = 0.30
    IR_SENSOR_ANGLE_STEP_DEG = 40.0
    
    MIN_LIDAR_RANGE = 0.05
    LIDAR_FRONT_WINDOW_RAD = math.radians(90)
    LIDAR_CLUSTER_GAP = 5
    LIDAR_DOWNSAMPLE_STEP = 5

    LIDAR_X_OFFSET = 0.17  
    LIDAR_MOUNT_YAW = math.pi

    APF_TANGENT_MULTIPLIER = 0.3
    APF_MAX_REP_GAIN = 4.0
    APF_LIDAR_IGNORE_DIST = 0.3

    WF_MIN_LIDAR_RANGE = 0.1
    WF_MAX_RANGE = 1.2
    WALL_DIST_BASE = 0.22

    WF_KP_X = 1.2
    WF_KP_THETA = 1.5
    WF_BASE_SPEED = 0.1

    ESCAPE_FRONT_CLEAR_DIST = 1.2
    ESCAPE_NARROW_WINDOW_RAD = math.radians(5)
    ESCAPE_GOAL_ANGLE_RAD = math.radians(20)
    
    ROT_SPEED_STUCK = 0.6
    WF_BASE_STUCK_TIMEOUT = 10.0
    STUCK_RADIUS = 0.2
    REQUIRED_STUCK_COUNT = 3
    MIN_PROGRESS = 0.05
    NO_PROGRESS_TIMEOUT = 3.0

    ROBOT_RADIUS = 0.225

    def __init__(self):
        super().__init__('apf_node')

        self._declare_and_load_parameters()
        self.add_on_set_parameters_callback(self.param_cb)

        self.x = self.y = self.theta = 0.0
        self.goal_x = self.goal_y = self.goal_theta = 0.0
        self.goal_received = False

        self.ir = None
        self.scan = None
        self.deadman_ok = False

        self.stuck_positions = []
        self.stuck_counter = 0
        self.last_goal_dist = None
        self.last_progress_time = self.get_clock().now()

        self.wall_follow_mode = False
        self.wall_follow_entry_dist = None
        self.wall_side = None
        self.last_wall_side = None
        
        self.last_wall_seen_time = self.get_clock().now()
        self.wall_follow_start_time = None
        self.wall_follow_start_pos = None

        self.min_dist_during_wf = float('inf')
        self.wf_last_progress_time = self.get_clock().now()
        self.wf_stuck_timeout = self.WF_BASE_STUCK_TIMEOUT

        self.last_normal = None
        
        self.wf_angle_rotation = False
        self.wf_rot_start_theta = 0.0
        self.wf_rot_dir = 1.0

        self.last_position_update_time = self.get_clock().now()
        self.last_tracked_pos = np.array([0.0, 0.0])

        self.memory_lidar = set()
        self.memory_lidar_pts = []
        
        self.memory_ir = set()
        self.memory_ir_pts = []
        
        self.memory_force_pts = []
        self.last_saved_force_pos = None

        self.viz_reset_requested = False

        self.vel_pub = self.create_publisher(Twist, 'cmd_vel_unfiltered', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'apf_viz/markers', 10)
        self.goal_pub = self.create_publisher(Pose2D, 'robotino/goal', 10)

        self.create_subscription(Float64MultiArray, 'robotino/odometry', self.odom_cb, 10)
        self.create_subscription(Float64MultiArray, 'robotino/distance_sensors', self.ir_cb, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_cb, 10)
        self.create_subscription(Pose2D, 'robotino/goal', self.goal_cb, 10)
        self.create_subscription(Bool, 'deadman_ok', self.deadman_cb, 10)

        self.clear_path_client = self.create_client(Trigger, 'clear_path')

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Uruchomiono węzeł realizujący algorytm APF z trybem ucieczki Wall Follow.")


    def _declare_and_load_parameters(self):
        params = [
            ('k_att', 0.3), ('kr_ir', 0.0), ('kr_lidar', 5.0),
            ('rep_range_ir', 0.4), ('rep_range_lidar', 1.0),
            ('max_lin_vel', 0.15), ('max_ang_vel', 0.6),
            ('k_theta', 1.5), ('eps_goal', 0.15), ('eps_angle', 0.2)
        ]
        for name, val in params:
            self.declare_parameter(name, val)
        self.update_params()

    def update_params(self):
        self.k_att      = self.get_parameter('k_att').value
        self.kr_ir      = self.get_parameter('kr_ir').value
        self.kr_lidar   = self.get_parameter('kr_lidar').value
        self.rep_ir     = self.get_parameter('rep_range_ir').value
        self.rep_lidar  = self.get_parameter('rep_range_lidar').value
        self.max_lin    = self.get_parameter('max_lin_vel').value
        self.max_ang    = self.get_parameter('max_ang_vel').value
        self.k_theta    = self.get_parameter('k_theta').value
        self.eps_goal   = self.get_parameter('eps_goal').value
        self.eps_angle  = self.get_parameter('eps_angle').value

    def param_cb(self, params):
        self.update_params()
        return SetParametersResult(successful=True)

    def odom_cb(self, msg):
        if len(msg.data) >= 3:
            self.x, self.y, self.theta = msg.data[:3]

    def ir_cb(self, msg):
        if len(msg.data) == self.IR_SENSOR_COUNT:
            self.ir = np.array(msg.data)

    def scan_cb(self, msg):
        self.scan = msg

    def goal_cb(self, msg):
        if self.goal_received and math.isclose(msg.x, self.goal_x, abs_tol=1e-4) and math.isclose(msg.y, self.goal_y, abs_tol=1e-4):
            return

        self.goal_x, self.goal_y = msg.x, msg.y
        self.goal_theta = math.radians(msg.theta)
        self.goal_received = True
        
        self.last_goal_dist = None
        self.last_progress_time = self.get_clock().now()
        self.stuck_counter = 0
        self.last_wall_side = None
        self.wf_stuck_timeout = self.WF_BASE_STUCK_TIMEOUT
        self.reset_wf_state()

        self.memory_lidar.clear()
        self.memory_lidar_pts.clear()
        self.memory_ir.clear()
        self.memory_ir_pts.clear()
        self.memory_force_pts.clear()
        self.last_saved_force_pos = None
        self.viz_reset_requested = True

        self.call_clear_path_service()
        self.goal_pub.publish(msg)

        self.get_logger().info(f"Otrzymano nowy cel! x={self.goal_x:.2f}, y={self.goal_y:.2f}. Czyszczę starą ścieżkę...")

    def call_clear_path_service(self):
        if not self.clear_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Usługa czyszczenia ścieżki jest niedostępna!')
        else:
            req = Trigger.Request()
            future = self.clear_path_client.call_async(req)
            future.add_done_callback(self.clear_path_done_cb)

    def clear_path_done_cb(self, future):
        try:
            response = future.result()
            self.get_logger().info('Ścieżka została pomyślnie wyczyszczona.')
        except Exception as e:
            self.get_logger().error(f'Błąd podczas czyszczenia ścieżki: {e}')

    def deadman_cb(self, msg):
        self.deadman_ok = msg.data

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def orientation_error(self, theta, theta_d):
        e = self.normalize_angle(theta_d - theta)
        return 0.0 if abs(e) < self.eps_angle else e

    def attractive_force(self):
        return self.k_att * np.array([[self.goal_x - self.x], [self.goal_y - self.y]])

    def ir_rep_force(self):
        F = np.zeros((2, 1))
        if self.ir is None:
            return F, []
        
        dists = np.clip(self.ir, self.MIN_IR_DIST, None)
        valid_mask = (dists < self.rep_ir) & (dists <= self.MAX_IR_DIST)
        
        if not np.any(valid_mask):
            return F, []

        sensor_angles = self.theta + np.deg2rad(np.arange(self.IR_SENSOR_COUNT) * self.IR_SENSOR_ANGLE_STEP_DEG)
        
        valid_indices = np.where(valid_mask)[0]
        points = []
        
        for i in valid_indices:
            d = dists[i]
            ang = sensor_angles[i]
            
            obstacle_dist_from_center = self.ROBOT_RADIUS + d
            
            ox = self.x + obstacle_dist_from_center * math.cos(ang)
            oy = self.y + obstacle_dist_from_center * math.sin(ang)
            
            points.append(Point(x=ox, y=oy, z=0.0))

            dx, dy = self.x - ox, self.y - oy

            norm = math.hypot(dx, dy)
            if norm < 1e-5:
                continue

            nx, ny = dx / norm, dy / norm
            gain = min(self.kr_ir * (1 / d - 1 / self.rep_ir) / (d * d), self.APF_MAX_REP_GAIN)

            Fn = np.array([[nx], [ny]])
            Ft = np.array([[-ny], [nx]])

            cross = nx * (self.goal_y - oy) - ny * (self.goal_x - ox)
            sign = np.sign(cross) if cross != 0 else 1.0
            
            F += gain * (Fn + self.APF_TANGENT_MULTIPLIER * sign * Ft)

        return F, points

    def lidar_rep_force(self):
        RF = np.zeros((2, 1))
        points, centers = [], []
        if self.scan is None:
            return RF, points, centers

        dist_goal = math.hypot(self.goal_x - self.x, self.goal_y - self.y)
        if dist_goal < self.APF_LIDAR_IGNORE_DIST:
            return RF, points, centers

        ranges = np.array(self.scan.ranges)
        mid = len(ranges) // 2
        window = int(self.LIDAR_FRONT_WINDOW_RAD / self.scan.angle_increment)

        start_idx, end_idx = max(0, mid - window), min(len(ranges), mid + window)

        idx_slice = np.arange(start_idx, end_idx)
        r_slice = ranges[idx_slice]
        
        valid_mask = np.isfinite(r_slice) & (r_slice < self.rep_lidar)
        valid_idx = idx_slice[valid_mask]
        
        if len(valid_idx) == 0:
            return RF, points, centers

        raw_angles = self.scan.angle_min + valid_idx * self.scan.angle_increment
        local_angles = raw_angles + self.LIDAR_MOUNT_YAW
        
        r = ranges[valid_idx]
        x_robot = self.LIDAR_X_OFFSET + r * np.cos(local_angles)
        y_robot = 0.0 + r * np.sin(local_angles)

        xs = self.x + (x_robot * math.cos(self.theta) - y_robot * math.sin(self.theta))
        ys = self.y + (x_robot * math.sin(self.theta) + y_robot * math.cos(self.theta))

        for x, y in zip(xs[::self.LIDAR_DOWNSAMPLE_STEP], ys[::self.LIDAR_DOWNSAMPLE_STEP]):
            points.append(Point(x=float(x), y=float(y), z=0.0))

        group_indices = np.split(np.arange(len(valid_idx)), np.where(np.diff(valid_idx) > self.LIDAR_CLUSTER_GAP)[0] + 1)

        for g_idx in group_indices:
            if len(g_idx) < 2: continue
            
            xo = np.mean(xs[g_idx])
            yo = np.mean(ys[g_idx])
            centers.append(Point(x=float(xo), y=float(yo), z=0.0))

            d = max(math.hypot(self.x - xo, self.y - yo), self.MIN_LIDAR_RANGE)
            gain = self.kr_lidar * (1 / d - 1 / self.rep_lidar) / (d * d)

            nx, ny = (self.x - xo) / d, (self.y - yo) / d
            Fn = np.array([[nx], [ny]])
            Ft = np.array([[-ny], [nx]])

            cross = nx * (self.goal_y - yo) - ny * (self.goal_x - xo)
            sign = np.sign(cross) if cross != 0 else 1.0
            
            RF += gain * (Fn + self.APF_TANGENT_MULTIPLIER * sign * Ft)

        return RF, points, centers

    def _handle_wf_rotation(self, now):
        self.last_wall_seen_time = now
        diff = self.orientation_error(self.theta, self.wf_rot_start_theta)
        
        if abs(diff) < (math.pi / 2.0):
            twist = Twist()
            twist.angular.z = self.wf_rot_dir * self.ROT_SPEED_STUCK
            return twist
            
        self.wf_angle_rotation = False
        self._update_tracking_position(now)
        return None

    def _update_tracking_position(self, now):
        self.last_position_update_time = now
        self.last_tracked_pos = np.array([self.x, self.y])

    def wall_follow_control(self):
        twist = Twist()
        now = self.get_clock().now()

        if self.wf_angle_rotation:
            rot_twist = self._handle_wf_rotation(now)
            if rot_twist: return rot_twist

        if self.scan is None:
            return twist

        ranges = np.array(self.scan.ranges)
        n = len(ranges)
        mid = n // 2

        idx = int(math.radians(80) / self.scan.angle_increment)
        start_idx = max(0, mid - idx)
        end_idx = min(n, mid + idx)

        front_indices = np.arange(start_idx, end_idx)
        front_ranges = ranges[front_indices]

        valid_mask = np.isfinite(front_ranges) & (front_ranges > self.WF_MIN_LIDAR_RANGE)
        valid_front_indices = front_indices[valid_mask]

        if len(valid_front_indices) == 0:
            twist.linear.x = self.WF_BASE_SPEED * 0.5
            return twist

        min_idx = valid_front_indices[np.argmin(ranges[valid_front_indices])]
        min_dist = ranges[min_idx]

        if min_dist > self.WF_MAX_RANGE:
            twist.linear.x = self.WF_BASE_SPEED
            return twist

        angle_error = (min_idx - mid) * self.scan.angle_increment
        wall_angle_world = self.theta + angle_error
        normal = np.array([-math.cos(wall_angle_world), -math.sin(wall_angle_world)])
        self.last_normal = normal

        if self.wall_side is None:
            if self.last_wall_side is not None:
                self.wall_side = 'right' if self.last_wall_side == 'left' else 'left'
            else:
                goal_vec = np.array([self.goal_x - self.x, self.goal_y - self.y])
                t_left = np.array([-normal[1], normal[0]])
                t_right = np.array([normal[1], -normal[0]])
                self.wall_side = 'left' if np.dot(t_left, goal_vec) > np.dot(t_right, goal_vec) else 'right'
            self.last_wall_side = self.wall_side

        twist.angular.z = float(np.clip(self.WF_KP_THETA * angle_error, -self.max_ang, self.max_ang))
        distance_error = min_dist - self.WALL_DIST_BASE
        twist.linear.x = float(np.clip(self.WF_KP_X * distance_error, -self.max_lin, self.max_lin))

        if self.wall_side == 'left':
            twist.linear.y = self.WF_BASE_SPEED
        else:
            twist.linear.y = -self.WF_BASE_SPEED

        if abs(angle_error) > math.radians(15):
            twist.linear.y *= 0.2

        robot_pos = np.array([self.x, self.y])
        if np.linalg.norm(robot_pos - self.last_tracked_pos) > 0.01:
            self._update_tracking_position(now)
        else:
            time_stuck = (now - self.last_position_update_time).nanoseconds * 1e-9
            if time_stuck > 1.0:
                self.get_logger().warn("Wykryto narożnik wewnętrzny / utknięcie -> Obrót o 90 stopni.")
                self.wf_angle_rotation = True
                self.wf_rot_start_theta = self.theta
                self.wf_rot_dir = -1.0 if self.wall_side == 'right' else 1.0
                
                twist = Twist()
                twist.angular.z = self.wf_rot_dir * self.ROT_SPEED_STUCK
                return twist

        self.last_wall_seen_time = now
        return twist

    def check_wall_follow_trigger(self, dist):
        now = self.get_clock().now()
        elapsed = (now - self.last_progress_time).nanoseconds * 1e-9

        if self.last_goal_dist is None:
            self.last_goal_dist = dist
            return

        progress = self.last_goal_dist - dist
        if progress > self.MIN_PROGRESS:
            self.last_goal_dist = dist
            self.last_progress_time = now
            return

        if elapsed > self.NO_PROGRESS_TIMEOUT:
            current_pos = np.array([self.x, self.y])
            repeated_stuck = sum(1 for p in self.stuck_positions if np.linalg.norm(current_pos - p) < self.STUCK_RADIUS)

            self.stuck_positions.append(current_pos)
            self.last_progress_time = now
            self.last_goal_dist = dist

            if repeated_stuck + 1 >= self.REQUIRED_STUCK_COUNT:
                self._activate_wall_following(dist, now, current_pos)

    def _activate_wall_following(self, dist, now, current_pos):
        self.wall_follow_mode = True
        self.wall_follow_entry_dist = dist
        self.wall_side = None
        self.min_dist_during_wf = dist
        self.wf_last_progress_time = now
        self.wall_follow_start_time = now
        self.wall_follow_start_pos = current_pos
        self.stuck_counter += 1
        self.last_wall_seen_time = now
        
        self.wf_angle_rotation = False
        self._update_tracking_position(now)

        if self.last_wall_side is not None:
            self.wf_stuck_timeout = self.WF_BASE_STUCK_TIMEOUT + 10.0
        else:
            self.wf_stuck_timeout = self.WF_BASE_STUCK_TIMEOUT

    def should_exit_wf_due_to_stuck(self, current_dist):
        if self.wall_follow_start_time is None or self.wf_angle_rotation:
            return False
        
        now = self.get_clock().now()
        if current_dist < self.min_dist_during_wf - 0.05:
            self.min_dist_during_wf = current_dist
            self.wf_last_progress_time = now
        else:
            wf_stuck_duration = (now - self.wf_last_progress_time).nanoseconds * 1e-9
            if wf_stuck_duration > self.wf_stuck_timeout:
                return True
        return False

    def reset_wf_state(self):
        self.wall_follow_mode = False
        self.wall_follow_entry_dist = None
        self.wf_angle_rotation = False
        self.last_normal = None
        self.stuck_positions.clear()

    def control_loop(self):
        if not self.goal_received:
            return

        AF = self.attractive_force()
        RF_ir, ir_points = self.ir_rep_force()
        RF_lidar, lidar_points, _ = self.lidar_rep_force()
        
        RF = RF_ir + RF_lidar

        self.visualize(AF, RF, ir_points, lidar_points)

        dist = math.hypot(self.goal_x - self.x, self.goal_y - self.y)
        
        if not self.wall_follow_mode and dist >= self.eps_goal:
            if self.deadman_ok:
                self.check_wall_follow_trigger(dist)
            else:
                self.last_progress_time = self.get_clock().now()

        if self.wall_follow_mode:
            self._handle_wall_follow_state(dist, AF, RF)
        else:
            self.execute_apf(AF, RF, dist)

    def _handle_wall_follow_state(self, dist, AF, RF):
        if self.scan is not None:
            ranges = np.array(self.scan.ranges)
            mid = len(ranges) // 2
            nar_win = int(self.ESCAPE_NARROW_WINDOW_RAD / self.scan.angle_increment)
            valid_narrow = ranges[mid - nar_win : mid + nar_win]
            valid_narrow = valid_narrow[np.isfinite(valid_narrow)]
            
            if len(valid_narrow) > 0:
                avg_narrow = np.mean(valid_narrow)
                angle_diff = self.normalize_angle(math.atan2(self.goal_y - self.y, self.goal_x - self.x) - self.theta)
                
                if avg_narrow > self.ESCAPE_FRONT_CLEAR_DIST and abs(angle_diff) < self.ESCAPE_GOAL_ANGLE_RAD:
                    self.reset_wf_state()
                    return self.execute_apf(AF, RF, dist)

        if self.should_exit_wf_due_to_stuck(dist):
            self.reset_wf_state()
            return self.execute_apf(AF, RF, dist)

        twist = self.wall_follow_control()
        now = self.get_clock().now()
        wall_lost_time = (now - self.last_wall_seen_time).nanoseconds * 1e-9

        if self.wall_follow_entry_dist is not None:
            dist_improved = dist < (self.wall_follow_entry_dist - 0.15)
            
            goal_vec = np.array([self.goal_x - self.x, self.goal_y - self.y])
            goal_norm = np.linalg.norm(goal_vec)
            if goal_norm > 0:
                goal_vec = goal_vec / goal_norm

            path_clear = np.dot(RF.flatten(), goal_vec) > -0.1

            wall_in_the_way = False
            if self.last_normal is not None:
                wall_in_the_way = np.dot(self.last_normal, goal_vec) > 0.1

            if (dist_improved and path_clear and not wall_in_the_way) or wall_lost_time > 2.0:
                self.reset_wf_state()

        self.vel_pub.publish(twist)

    def execute_apf(self, AF, RF, dist):
        F = AF + RF
        twist = Twist()

        if dist < self.eps_goal:
            e = self.orientation_error(self.theta, self.goal_theta)
        else:
            theta_d = math.atan2(F[1, 0], F[0, 0])
            e = self.orientation_error(self.theta, theta_d)

        twist.angular.z = float(np.clip(self.k_theta * e, -self.max_ang, self.max_ang))

        fx = F[0, 0] * math.cos(self.theta) + F[1, 0] * math.sin(self.theta)
        fy = -F[0, 0] * math.sin(self.theta) + F[1, 0] * math.cos(self.theta)
        mag = math.hypot(fx, fy)

        if mag > 1e-3 and dist >= self.eps_goal:
            v = min(self.max_lin, mag)
            speed_scale = max(0.5, 1.0 - 0.3 * abs(e))
            twist.linear.x = float(fx / mag * v * speed_scale)
            twist.linear.y = float(fy / mag * v * speed_scale)

        self.vel_pub.publish(twist)

    
    def visualize(self, AF, RF, ir_points, lidar_points):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        current_pos = np.array([self.x, self.y])

        if self.viz_reset_requested:
            clear = Marker()
            clear.action = Marker.DELETEALL
            markers.markers.append(clear)
            self.viz_reset_requested = False

        def create_base_marker(m_id, m_type):
            m = Marker()
            m.header.frame_id = "odom"
            m.header.stamp = now
            m.ns = "apf"
            m.id = m_id
            m.type = m_type
            m.action = Marker.ADD
            return m

        for p in lidar_points:
            p_tuple = (round(p.x, 2), round(p.y, 2))
            if p_tuple not in self.memory_lidar:
                self.memory_lidar.add(p_tuple)
                self.memory_lidar_pts.append(Point(x=p_tuple[0], y=p_tuple[1], z=0.0))

        for p in ir_points:
            p_tuple = (round(p.x, 2), round(p.y, 2))
            if p_tuple not in self.memory_ir:
                self.memory_ir.add(p_tuple)
                self.memory_ir_pts.append(Point(x=p_tuple[0], y=p_tuple[1], z=0.0))

        if self.memory_lidar_pts:
            m_lidar = create_base_marker(1, Marker.POINTS)
            m_lidar.scale.x = m_lidar.scale.y = 0.02
            m_lidar.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
            m_lidar.points = self.memory_lidar_pts
            markers.markers.append(m_lidar)

        if self.memory_ir_pts:
            m_ir = create_base_marker(2, Marker.POINTS)
            m_ir.scale.x = m_ir.scale.y = 0.06
            m_ir.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
            m_ir.points = self.memory_ir_pts
            markers.markers.append(m_ir)

        res_force = AF + RF
        res_fx, res_fy = float(res_force[0, 0]), float(res_force[1, 0])
        
        if self.last_saved_force_pos is None or np.linalg.norm(current_pos - self.last_saved_force_pos) > 0.10:
            if math.hypot(res_fx, res_fy) > 0.01:
                scale_factor = 0.3
                p_start = Point(x=self.x, y=self.y, z=0.01)
                p_end = Point(x=self.x + res_fx * scale_factor, y=self.y + res_fy * scale_factor, z=0.01)
                self.memory_force_pts.extend([p_start, p_end])
                self.last_saved_force_pos = current_pos

        if self.stuck_positions:
            m_stuck = create_base_marker(4, Marker.SPHERE_LIST)
            m_stuck.scale.x = m_stuck.scale.y = m_stuck.scale.z = 0.2
            m_stuck.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)
            m_stuck.points = [Point(x=float(p[0]), y=float(p[1]), z=0.1) for p in self.stuck_positions]
            markers.markers.append(m_stuck)

        def add_arrow(m_id, color, vector, scale_factor=0.3):
            mag = math.hypot(vector[0, 0], vector[1, 0])
            if mag < 0.01: return
            m = create_base_marker(m_id, Marker.ARROW)
            m.scale.x, m.scale.y, m.scale.z = 0.02, 0.04, 0.0
            m.color = color
            p_end = Point(x=self.x + float(vector[0, 0])*scale_factor, y=self.y + float(vector[1, 0])*scale_factor, z=0.1)
            m.points = [Point(x=self.x, y=self.y, z=0.1), p_end]
            markers.markers.append(m)

        add_arrow(5, ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0), AF)
        add_arrow(6, ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), RF)
        add_arrow(7, ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0), res_force)

        self.marker_pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = APFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()