#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool, Float64MultiArray, String
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
import subprocess
import sys
import time
import signal
import select
import termios
import tty
from collections import deque

TARGET_IP = "192.168.0.100"
CHECK_INTERVAL = 0.2
PING_WINDOW_SIZE = 10
CONSECUTIVE_FAILS_LIMIT = 5

IR_SENSOR_COUNT = 9


class SafetyNode(Node):

    def __init__(self):
        super().__init__('safety_node')

        self.declare_parameter('ir_safe_distance', 0.20)
        self.declare_parameter('ir_stop_distance', 0.10)
        self.declare_parameter('ir_stop_duration', 0.5)
        self.declare_parameter('bumper_stop_duration', 0.5)
        self.declare_parameter('velocity_watchdog_timeout', 2.0)
        
        self.update_params()
        self.add_on_set_parameters_callback(self.param_cb)

        self.deadman_pub = self.create_publisher(Bool, "deadman_ok", 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'safety_status', 10)

        self.bumper_sub = self.create_subscription(Bool, "robotino/bumper", self.bumper_callback, 10)
    
        self.distance_sub = self.create_subscription(Float64MultiArray, "robotino/distance_sensors", self.distance_callback, 10)

        self.vel_sub = self.create_subscription(Twist, 'cmd_vel_unfiltered', self.vel_callback, 10)

        self.timer = self.create_timer(CHECK_INTERVAL, self.loop)

        self.armed = False
        self.collision_latched = False
        self.shutdown_flag = False

        self.ping_history = deque(maxlen=PING_WINDOW_SIZE)

        self.stop_until = 0.0
        self.last_ir = [1.0] * IR_SENSOR_COUNT
        self.front_blocked = False
        self.rear_blocked = False
        self.left_blocked = False
        self.right_blocked = False

        self.last_vel_time = self.get_clock().now()
        self.vel_watchdog_triggered = False

        if sys.stdin.isatty():
            self.settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            self.keyboard_enabled = True
        else:
            self.get_logger().warning("Brak TTY - klawiatura nieaktywna")
            self.settings = None
            self.keyboard_enabled = False

        self.get_logger().warning("=== SAFETY NODE AKTYWNY ===")
        self.get_logger().warning("ENTER = START | SPACJA = STOP")
        self.get_logger().info(f"IR Safe: {self.ir_safe_distance}m | Stop: {self.ir_stop_distance}m")

    def update_params(self):
        self.ir_safe_distance = self.get_parameter('ir_safe_distance').value
        self.ir_stop_distance = self.get_parameter('ir_stop_distance').value
        self.ir_stop_duration = self.get_parameter('ir_stop_duration').value
        self.bumper_stop_duration = self.get_parameter('bumper_stop_duration').value
        self.vel_watchdog_timeout = self.get_parameter('velocity_watchdog_timeout').value

    def param_cb(self, params):
        self.update_params()
        self.get_logger().info(f"Parametry zaktualizowane: Safe={self.ir_safe_distance}, Stop={self.ir_stop_distance}")
        return SetParametersResult(successful=True)

    def bumper_callback(self, msg: Bool):
        if msg.data and not self.collision_latched:
            self.get_logger().error("╔════════════════════════════╗")
            self.get_logger().error("║  BUMPER - EMERGENCY STOP   ║")
            self.get_logger().error("╚════════════════════════════╝")
            self.get_logger().warning("Naciśnij ENTER aby zresetować i wznowić")

            self.collision_latched = True
            self.armed = False
            self.stop_until = time.time() + self.bumper_stop_duration
            
            self.publish_status("BUMPER_EMERGENCY_STOP")

    def distance_callback(self, msg: Float64MultiArray):
        if len(msg.data) != IR_SENSOR_COUNT:
            self.get_logger().warning(f"Nieprawidłowa liczba czujników IR: {len(msg.data)}")
            return

        self.last_ir = list(msg.data)

        front = self.last_ir[0]
        left = min(self.last_ir[1:4])
        rear = min(self.last_ir[4:6])
        right = min(self.last_ir[6:9])

        self.front_blocked = front < self.ir_safe_distance
        self.rear_blocked = rear < self.ir_safe_distance
        self.left_blocked = left < self.ir_safe_distance
        self.right_blocked = right < self.ir_safe_distance

        min_dist = min(self.last_ir)
        
        if min_dist < self.ir_stop_distance:
            now = time.time()
            if now >= self.stop_until:
                self.get_logger().warning(
                    f"HARD STOP: Przeszkoda {min_dist*100:.1f}cm (limit {self.ir_stop_distance*100:.1f}cm)"
                )
                self.publish_status(f"IR_HARD_STOP_{min_dist:.3f}m")
            
            self.stop_until = now + self.ir_stop_duration

    def vel_callback(self, msg: Twist):
        self.last_vel_time = self.get_clock().now()
        self.vel_watchdog_triggered = False

        if not self.armed or self.collision_latched:
            self.vel_pub.publish(Twist())
            return

        front = self.last_ir[0]
        left = min(self.last_ir[1:4])
        rear = min(self.last_ir[4:6])
        right = min(self.last_ir[6:9])
        min_dist = min(self.last_ir)

        if min_dist < self.ir_stop_distance:
            return

        filtered = Twist()
        vx, vy = msg.linear.x, msg.linear.y
        filtered.angular.z = msg.angular.z

        def get_scale(dist):
            s = (dist - self.ir_stop_distance) / (self.ir_safe_distance - self.ir_stop_distance)
            return max(0.0, min(1.0, s))

        if vx > 0 and front < self.ir_safe_distance:
            vx *= get_scale(front)
        elif vx < 0 and rear < self.ir_safe_distance:
            vx *= get_scale(rear)

        if vy > 0 and left < self.ir_safe_distance:
            vy *= get_scale(left)
        elif vy < 0 and right < self.ir_safe_distance:
            vy *= get_scale(right)

        filtered.linear.x = vx
        filtered.linear.y = vy
        
        self.vel_pub.publish(filtered)

    def get_key(self, timeout=0.0):
        if not self.keyboard_enabled:
            return None

        rlist, _, _ = select.select([sys.stdin], [], [], timeout)

        if rlist:
            return sys.stdin.read(1)

        return None

    def handle_enter(self):
        if self.collision_latched:
            self.get_logger().warning("RESET KOLIZJI + START")
            self.collision_latched = False
            self.ping_history.clear()
            self.stop_until = 0.0
            self.armed = True
            self.publish_status("COLLISION_RESET_AND_ARMED")
            return

        if not self.armed:
            self.get_logger().warning("DEADMAN UZBROJONY")
            self.ping_history.clear()
            self.armed = True
            self.publish_status("ARMED")

    def handle_space(self):
        if self.armed:
            self.get_logger().warning("STOP (operator)")
            self.armed = False
            self.publish_status("OPERATOR_STOP")

    def single_ping_ok(self) -> bool:
        try:
            result = subprocess.run(
                ["ping", "-c", "1", "-W", "1", TARGET_IP],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            return result.returncode == 0

        except subprocess.SubprocessError:
            return False

    def has_consecutive_fails(self) -> bool:
        count = 0

        for result in self.ping_history:
            if not result:
                count += 1
                if count >= CONSECUTIVE_FAILS_LIMIT:
                    return True
            else:
                count = 0

        return False

    def check_velocity_watchdog(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_vel_time).nanoseconds * 1e-9

        if elapsed > self.vel_watchdog_timeout and not self.vel_watchdog_triggered:
            self.get_logger().error(
                f"WATCHDOG: Brak cmd_vel_unfiltered przez {elapsed:.1f}s"
            )
            self.vel_watchdog_triggered = True
            self.publish_status("VELOCITY_WATCHDOG_TIMEOUT")

    def publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def loop(self):
        key = self.get_key()

        if key == '\r' or key == '\n':
            self.handle_enter()
        elif key == ' ':
            self.handle_space()

        ping_ok = self.single_ping_ok()
        self.ping_history.append(ping_ok)

        network_ok = True
        if len(self.ping_history) >= CONSECUTIVE_FAILS_LIMIT:
            network_ok = not self.has_consecutive_fails()

        if not network_ok and self.armed:
            self.get_logger().error("Brak operatora (sieć) -> STOP")
            self.armed = False
            self.publish_status("NETWORK_LOST")

        self.check_velocity_watchdog()

        safety_ok = time.time() >= self.stop_until
        min_dist = min(self.last_ir)

        if min_dist < self.ir_stop_distance and self.armed and not self.collision_latched:
            escape_vel = Twist()
            front = self.last_ir[0]
            left = min(self.last_ir[1:4])
            rear = min(self.last_ir[4:6])
            right = min(self.last_ir[6:9])
            
            if front < self.ir_stop_distance:
                escape_vel.linear.x = -0.08
            elif rear < self.ir_stop_distance:
                escape_vel.linear.x = 0.08
                
            if left < self.ir_stop_distance:
                escape_vel.linear.y = -0.08
            elif right < self.ir_stop_distance:
                escape_vel.linear.y = 0.08

            self.get_logger().info("TRYB UCIECZKI: Autonomiczne odsuwanie od przeszkody", throttle_duration_sec=1.0)
            self.vel_pub.publish(escape_vel)

        if self.collision_latched:
            deadman_ok = False
            status = "COLLISION_LATCHED"
        else:
            deadman_ok = self.armed and network_ok
            
            if not self.armed:
                status = "DISARMED"
            elif not network_ok:
                status = "NETWORK_FAIL"
            elif not safety_ok:
                status = "IR_SAFETY_BLOCK"
            else:
                status = "OK"

        msg = Bool()
        msg.data = deadman_ok
        self.deadman_pub.publish(msg)

        if not hasattr(self, '_last_status') or self._last_status != status:
            self.publish_status(status)
            self._last_status = status

    def shutdown_deadman(self):
        self.get_logger().warning("=== SHUTDOWN SAFETY NODE ===")

        msg = Bool()
        msg.data = False

        for _ in range(5):
            self.deadman_pub.publish(msg)
            time.sleep(0.02)

        stop = Twist()
        for _ in range(5):
            self.vel_pub.publish(stop)
            time.sleep(0.02)

        if self.settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        self.publish_status("SHUTDOWN")
        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()

    node = SafetyNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def sigint_handler(sig, frame):
        node.shutdown_flag = True

    signal.signal(signal.SIGINT, sigint_handler)

    try:
        while rclpy.ok() and not node.shutdown_flag:
            executor.spin_once(timeout_sec=0.1)

    finally:
        node.shutdown_deadman()


if __name__ == "__main__":
    main()