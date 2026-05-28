#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import Trigger

import math
import numpy as np

class Bug2Node(Node):

    MIN_LIDAR_RANGE = 0.18
    WF_MAX_RANGE = 1.2

    WF_KP_X = 1.2
    WF_KP_THETA = 1.5
    WF_BASE_SPEED = 0.1
    WF_BASE_STUCK_TIMEOUT = 20.0

    ROT_SPEED_STUCK = 0.6

    LIDAR_FRONT_WINDOW_RAD = math.radians(90)
    LIDAR_DOWNSAMPLE_STEP = 5
    LIDAR_X_OFFSET = 0.17
    LIDAR_MOUNT_YAW = math.pi

    def __init__(self):
        super().__init__('bug2_node')

        self._declare_and_load_parameters()
        self.add_on_set_parameters_callback(self.param_cb)

        self.x = self.y = self.theta = 0.0
        self.start_x = self.start_y = None
        self.goal_x = self.goal_y = self.goal_theta = 0.0
        self.goal_received = False
        
        self.mode = "GO_TO_GOAL"
        self.min_dist_to_goal_seen = float('inf')
        
        self.scan = None
        self.ir = None
        self.obstacle_points_odom = []
        
        self.is_rotating = False
        
        self.hit_point = None
        self.leave_point = None
        self.hit_dist = None
        self.last_wall_follow_exit = None
        self.ignore_obstacles_time = 2.0

        self.current_wall_side = None

        self.last_position_update_time = self.get_clock().now()
        self.last_tracked_pos = np.array([0.0, 0.0])
        
        self.min_dist_during_wf = float('inf')
        self.wf_last_progress_time = self.get_clock().now()

        self.wf_angle_rotation = False
        self.wf_rot_start_theta = 0.0
        self.wf_rot_dir = 1.0

        self.memory_lidar = set()
        self.memory_lidar_pts = []
        self.viz_reset_requested = False

        self.vel_pub = self.create_publisher(Twist, 'cmd_vel_unfiltered', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'bug2_viz/markers', 10)
        self.goal_pub = self.create_publisher(Pose2D, 'robotino/goal', 10)

        self.clear_path_client = self.create_client(Trigger, 'clear_path')

        self.create_subscription(Float64MultiArray, 'robotino/odometry', self.odom_cb, 10)
        self.create_subscription(Float64MultiArray, 'robotino/distance_sensors', self.ir_cb, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_cb, 10)
        self.create_subscription(Pose2D, 'robotino/goal', self.goal_cb, 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Uruchomiono węzeł realizujący algorytm Bug2.')


    def _declare_and_load_parameters(self):
        params = [
            ('obstacle_threshold', 0.5), 
            ('odom_detect_distance', 0.3),
            ('wall_distance', 0.25),
            ('wall_follow_side', 'left'),
            ('max_lin_vel', 0.15), 
            ('max_ang_vel', 0.4),
            ('eps_goal', 0.2), 
            ('eps_angle', 0.2)
        ]
        for name, val in params:
            self.declare_parameter(name, val)
        self.update_params()

    def update_params(self):
        self.obst_thresh = self.get_parameter('obstacle_threshold').value
        self.odom_detect_dist = self.get_parameter('odom_detect_distance').value
        self.wall_dist = self.get_parameter('wall_distance').value
        self.wall_follow_side = self.get_parameter('wall_follow_side').value
        self.lin_vel = self.get_parameter('max_lin_vel').value
        self.ang_vel = self.get_parameter('max_ang_vel').value
        self.eps_goal = self.get_parameter('eps_goal').value
        self.eps_angle = self.get_parameter('eps_angle').value

    def param_cb(self, params):
        for p in params:
            if hasattr(self, p.name):
                setattr(self, p.name, p.value)
        self.update_params()
        return SetParametersResult(successful=True)

    def call_clear_path_service(self):
        if not self.clear_path_client.service_is_ready():
            self.get_logger().warn("Usługa 'clear_path' nie jest jeszcze dostępna. Pomijam.")
            return
        req = Trigger.Request()
        self.clear_path_client.call_async(req)
        goal_msg = Pose2D()
        goal_msg.x = float(self.goal_x)
        goal_msg.y = float(self.goal_y)
        self.goal_pub.publish(goal_msg)
        self.get_logger().info("Wysłano żądanie wyczyszczenia ścieżki (clear_path).")

    def odom_cb(self, msg):
        if len(msg.data) >= 3:
            self.x, self.y, self.theta = msg.data[:3]
            if self.start_x is None:
                self.start_x, self.start_y = self.x, self.y

    def ir_cb(self, msg):
        if len(msg.data) == 9:
            self.ir = msg.data

    def scan_cb(self, msg):
        self.scan = msg
        self.detect_obstacles()

    def goal_cb(self, msg: Pose2D):
        if self.goal_received and math.isclose(msg.x, self.goal_x, abs_tol=1e-4) and math.isclose(msg.y, self.goal_y, abs_tol=1e-4):
            return

        self.goal_x, self.goal_y = msg.x, msg.y
        self.goal_theta = math.radians(msg.theta)
        self.goal_received = True
        
        self.mode = "GO_TO_GOAL"
        self.hit_point = None
        self.leave_point = None
        self.obstacle_points_odom = []
        self.min_dist_to_goal_seen = float('inf')
        self.start_x = self.start_y = None
        
        self.memory_lidar.clear()
        self.memory_lidar_pts.clear()
        self.viz_reset_requested = True
        
        self.call_clear_path_service()
        self.goal_pub.publish(msg)
        
        self.get_logger().info(f"Nowy cel zaakceptowany: x={self.goal_x:.2f}, y={self.goal_y:.2f}")
        

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def orientation_error(self, theta, theta_d):
        e = self.normalize_angle(theta_d - theta)
        return 0.0 if abs(e) < self.eps_angle else e

    def distance_to_goal(self):
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    def distance_to_mline(self):
        if self.start_x is None or self.start_y is None:
            return float('inf')
        sx, sy = self.start_x, self.start_y
        gx, gy = self.goal_x, self.goal_y
        numerator = abs((gy - sy)*self.x - (gx - sx)*self.y + gx*sy - gy*sx)
        denominator = math.hypot(gy - sy, gx - sx)
        return numerator / denominator if denominator > 1e-6 else float('inf')

    def progress_along_mline(self, x, y):
        if self.start_x is None or self.start_y is None:
            return 0.0
        sx, sy = self.start_x, self.start_y
        gx, gy = self.goal_x, self.goal_y
        goal_vec = np.array([gx - sx, gy - sy])
        
        norm = np.linalg.norm(goal_vec)
        if norm < 1e-6:
            return 0.0
            
        robot_vec = np.array([x - sx, y - sy])
        return np.dot(robot_vec, goal_vec) / norm

    def detect_obstacles(self):
        self.obstacle_points_odom = []
        self.min_lidar_range = float('inf')
        self.min_odom_dist = float('inf')
        
        if self.scan is None:
            return
            
        ranges = np.array(self.scan.ranges)
        n = len(ranges)
        mid = n // 2
        
        window = int(math.radians(60) / self.scan.angle_increment)
        start = max(0, mid - window)
        end = min(n, mid + window)
        
        for i in range(start, end):
            r = ranges[i]
            if not math.isfinite(r) or r > self.obst_thresh or r < self.MIN_LIDAR_RANGE:
                continue
            if r < self.min_lidar_range:
                self.min_lidar_range = r
                
            raw_angle = self.scan.angle_min + i * self.scan.angle_increment
            local_angle = raw_angle + self.LIDAR_MOUNT_YAW

            x_robot = self.LIDAR_X_OFFSET + r * math.cos(local_angle)
            y_robot = 0.0 + r * math.sin(local_angle)

            x_o = self.x + (x_robot * math.cos(self.theta) - y_robot * math.sin(self.theta))
            y_o = self.y + (x_robot * math.sin(self.theta) + y_robot * math.cos(self.theta))
            
            self.obstacle_points_odom.append((x_o, y_o))
            
            d = math.hypot(self.x - x_o, self.y - y_o)
            if d < self.min_odom_dist:
                self.min_odom_dist = d

    def close_to_obstacle_points(self):
        closest = None
        min_dist = float('inf')
        for ox, oy in self.obstacle_points_odom:
            d = math.hypot(self.x - ox, self.y - oy)
            if d < min_dist:
                min_dist = d
                closest = (ox, oy)
        
        detection_threshold = self.odom_detect_dist + self.LIDAR_X_OFFSET
        return min_dist < detection_threshold, closest

    def saturate_twist(self, twist: Twist) -> Twist:
        twist.linear.x = float(np.clip(twist.linear.x, -self.lin_vel, self.lin_vel))
        twist.linear.y = float(np.clip(twist.linear.y, -self.lin_vel, self.lin_vel))
        twist.angular.z = float(np.clip(twist.angular.z, -self.ang_vel, self.ang_vel))
        return twist

    def go_to_goal_twist(self, goal_angle):
        twist = Twist()
        ang_err = self.orientation_error(self.theta, goal_angle)
        
        if abs(ang_err) > self.eps_angle:
            self.is_rotating = True
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = np.clip(self.ang_vel * (ang_err / abs(ang_err)), -self.ang_vel, self.ang_vel)
        else:
            self.is_rotating = False
            twist.linear.x = self.lin_vel
            twist.angular.z = 0.0
            
        return twist

    def align_to_goal_orientation(self):
        twist = Twist()
        rotating = False
        orient_err = self.orientation_error(self.theta, self.goal_theta)
        if abs(orient_err) > self.eps_angle:
            twist.angular.z = np.clip(self.ang_vel * (orient_err / abs(orient_err)), -self.ang_vel, self.ang_vel)
            rotating = True
        return twist, rotating

    def _update_tracking_position(self, now):
        self.last_position_update_time = now
        self.last_tracked_pos = np.array([self.x, self.y])

    def _handle_wf_rotation(self, now):
        diff = self.orientation_error(self.theta, self.wf_rot_start_theta)
        if abs(diff) < (math.pi / 2.0):
            twist = Twist()
            twist.angular.z = self.wf_rot_dir * self.ROT_SPEED_STUCK
            return twist
        self.wf_angle_rotation = False
        self._update_tracking_position(now)
        return None

    def wall_follow_control(self, now):
        twist = Twist()
        if self.wf_angle_rotation:
            rot_twist = self._handle_wf_rotation(now)
            if rot_twist: return rot_twist

        if self.scan is None: return twist

        ranges = np.array(self.scan.ranges)
        n = len(ranges)
        mid = n // 2

        idx = int(math.radians(90) / self.scan.angle_increment)
        start_idx = max(0, mid - idx)
        end_idx = min(n, mid + idx)

        front_indices = np.arange(start_idx, end_idx)
        front_ranges = ranges[front_indices]

        valid_mask = np.isfinite(front_ranges) & (front_ranges > self.MIN_LIDAR_RANGE)
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
        twist.angular.z = float(np.clip(self.WF_KP_THETA * angle_error, -self.ang_vel, self.ang_vel))

        distance_error = min_dist - self.wall_dist
        twist.linear.x = float(np.clip(self.WF_KP_X * distance_error, -self.lin_vel, self.lin_vel))

        if self.current_wall_side == 'left':
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
                self.get_logger().warn("Robot utknął -> obrót o 90 stopni")
                self.wf_angle_rotation = True
                self.wf_rot_start_theta = self.theta
                self.wf_rot_dir = -1.0 if self.current_wall_side == 'right' else 1.0
                twist = Twist()
                twist.angular.z = self.wf_rot_dir * self.ROT_SPEED_STUCK
                return twist
        return twist

    def _activate_wall_follow(self, hit_point, now):
        self.mode = "WALL_FOLLOW"
        self.hit_point = hit_point
        self.hit_dist = self.progress_along_mline(hit_point[0], hit_point[1])

        if self.wall_follow_side == 'auto':
            if self.scan:
                n = len(self.scan.ranges)
                mid = n // 2
                left_vals = [r for r in self.scan.ranges[mid + 10: mid + 60] if math.isfinite(r)]
                right_vals = [r for r in self.scan.ranges[mid - 60 : mid - 10] if math.isfinite(r)]
                left_dist = sum(left_vals)/len(left_vals) if left_vals else float('inf')
                right_dist = sum(right_vals)/len(right_vals) if right_vals else float('inf')
                self.current_wall_side = 'left' if left_dist > right_dist else 'right'
            else:
                self.current_wall_side = 'left'
        else:
            self.current_wall_side = self.wall_follow_side

        self.get_logger().info(f"Wykryto ścianę. Omijanie ze strony: {self.current_wall_side}.")
        self.min_dist_during_wf = float('inf')
        self.wf_last_progress_time = now
        self._update_tracking_position(now)

    def check_stuck_condition_and_switch_side(self, dist, now):
        if dist < self.min_dist_during_wf - 0.05:
            self.min_dist_during_wf = dist
            self.wf_last_progress_time = now
        else:
            wf_stuck_duration = (now - self.wf_last_progress_time).nanoseconds * 1e-9
            if wf_stuck_duration > self.WF_BASE_STUCK_TIMEOUT:
                self.current_wall_side = 'right' if self.current_wall_side == 'left' else 'left'
                self.get_logger().warn(f"Brak postępu. Zmiana boku na: {self.current_wall_side}.")
                self.wf_last_progress_time = now
                self.min_dist_during_wf = dist
                self._update_tracking_position(now)
                
    def control_loop(self):
        if self.start_x is None or not self.goal_received: 
            return
            
        dist, goal_angle = self.distance_to_goal()
        dist_to_line = self.distance_to_mline()
        obstacle_close, hit = self.close_to_obstacle_points()

        twist = Twist()
        now = self.get_clock().now()

        if dist < self.eps_goal:
            twist, _ = self.align_to_goal_orientation()
            
        elif self.mode == "GO_TO_GOAL":
            ignore_obst = False
            if self.last_wall_follow_exit is not None:
                dt = (now - self.last_wall_follow_exit).nanoseconds * 1e-9
                ignore_obst = dt < self.ignore_obstacles_time

            if obstacle_close and self.min_lidar_range < self.odom_detect_dist and not ignore_obst and not self.is_rotating:
                self._activate_wall_follow(hit, now)
            else:
                twist = self.go_to_goal_twist(goal_angle)

        elif self.mode == "WALL_FOLLOW":
            self.check_stuck_condition_and_switch_side(dist, now)
            
            current_progress = self.progress_along_mline(self.x, self.y)
            if dist_to_line < 0.05 and current_progress > self.hit_dist + 0.50:
                self.last_wall_follow_exit = now
                self.leave_point = (self.x, self.y)

                twist, rotating = self.align_to_goal_orientation()
                if rotating:
                    self.vel_pub.publish(twist)
                    self.visualize()
                    return

                self.get_logger().info("Powrót na M-linię -> GO_TO_GOAL")
                self.mode = "GO_TO_GOAL"
                self.wf_angle_rotation = False
                self.is_rotating = False
                twist = self.go_to_goal_twist(goal_angle) 
            else:
                twist = self.wall_follow_control(now)

        twist = self.saturate_twist(twist)
        self.vel_pub.publish(twist)
        self.visualize()

    def visualize(self):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        if self.viz_reset_requested:
            clear = Marker()
            clear.action = Marker.DELETEALL
            markers.markers.append(clear)
            self.viz_reset_requested = False

        def create_base_marker(m_id, m_type):
            m = Marker()
            m.header.frame_id = "odom"
            m.header.stamp = now
            m.ns = "bug2"
            m.id = m_id
            m.type = m_type
            m.action = Marker.ADD
            return m

        if self.scan:
            ranges = np.array(self.scan.ranges)
            mid = len(ranges) // 2
            window = int(self.LIDAR_FRONT_WINDOW_RAD / self.scan.angle_increment)
            start_idx, end_idx = max(0, mid - window), min(len(ranges), mid + window)
            
            idx_slice = np.arange(start_idx, end_idx, self.LIDAR_DOWNSAMPLE_STEP)
            r_slice = ranges[idx_slice]
            
            valid_mask = np.isfinite(r_slice) & (r_slice < self.obst_thresh) & (r_slice > self.MIN_LIDAR_RANGE)
            valid_idx = idx_slice[valid_mask]
            
            if len(valid_idx) > 0:
                raw_angles = self.scan.angle_min + valid_idx * self.scan.angle_increment
                local_angles = raw_angles + self.LIDAR_MOUNT_YAW
                r_vals = ranges[valid_idx]

                x_robot = self.LIDAR_X_OFFSET + r_vals * np.cos(local_angles)
                y_robot = 0.0 + r_vals * np.sin(local_angles)

                xs = self.x + (x_robot * math.cos(self.theta) - y_robot * math.sin(self.theta))
                ys = self.y + (x_robot * math.sin(self.theta) + y_robot * math.cos(self.theta))
                
                for x, y in zip(xs, ys):
                    p_tuple = (round(x, 2), round(y, 2))
                    if p_tuple not in self.memory_lidar:
                        self.memory_lidar.add(p_tuple)
                        self.memory_lidar_pts.append(Point(x=p_tuple[0], y=p_tuple[1], z=0.0))

        if self.memory_lidar_pts:
            m_lidar = create_base_marker(1, Marker.POINTS)
            m_lidar.scale.x = m_lidar.scale.y = 0.02
            m_lidar.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
            m_lidar.points = self.memory_lidar_pts
            markers.markers.append(m_lidar)

        if self.start_x is not None and self.start_y is not None:
            m_line = create_base_marker(0, Marker.LINE_STRIP)
            m_line.scale.x = 0.03
            m_line.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            m_line.points = [
                Point(x=float(self.start_x), y=float(self.start_y), z=0.0),
                Point(x=float(self.goal_x), y=float(self.goal_y), z=0.0)
            ]
            markers.markers.append(m_line)

        m_hit = create_base_marker(2, Marker.SPHERE)
        if self.hit_point is not None:
            m_hit.scale.x = m_hit.scale.y = m_hit.scale.z = 0.12
            m_hit.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)
            m_hit.pose.position = Point(x=self.hit_point[0], y=self.hit_point[1], z=0.06)
        else:
            m_hit.action = Marker.DELETE
        markers.markers.append(m_hit)

        m_leave = create_base_marker(3, Marker.SPHERE)
        if self.leave_point is not None:
            m_leave.scale.x = m_leave.scale.y = m_leave.scale.z = 0.12
            m_leave.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0)
            m_leave.pose.position = Point(x=self.leave_point[0], y=self.leave_point[1], z=0.06)
        else:
            m_leave.action = Marker.DELETE
        markers.markers.append(m_leave)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = Bug2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Zatrzymywanie węzła...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()