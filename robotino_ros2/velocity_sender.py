#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import requests
import time

URL = 'http://192.168.0.12'

class VelocitySender(Node):

    VELOCITY_TIMEOUT = 0.2
    SEND_PERIOD = 0.1
    NODE_CHECK_PERIOD = 1.0
    
    def __init__(self):
        super().__init__('vel_sender')

        self.vx = self.vy = self.omega = 0.0

        self.last_cmd_time = None
        
        self.safety_node_present = False
        self.last_node_check_time = 0.0
        
        self.deadman_ok = False

        self.create_subscription(Twist, 'cmd_vel', self.velocity_callback, 10)

        self.create_subscription(Bool, 'deadman_ok', self.deadman_callback, 10)

        self.timer = self.create_timer(self.SEND_PERIOD, self.send_velocity)

        self.get_logger().info('Uruchomiono węzeł przesyłający dane o prędkościach do Robotino.')

    def velocity_callback(self, msg: Twist):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.omega = msg.angular.z
        self.last_cmd_time = time.monotonic()

    def deadman_callback(self, msg: Bool):
        if msg.data != self.deadman_ok:
            if msg.data:
                self.get_logger().warn("Deadman OK - ruch dozwolony")
            else:
                self.get_logger().warn("Deadman STOP - wymuszone zerowanie")
        self.deadman_ok = msg.data
    
    def check_network_deadman_node(self):
        now = time.monotonic()
        if now - self.last_node_check_time > self.NODE_CHECK_PERIOD:
            node_names = self.get_node_names()
            was_present = self.safety_node_present
            self.safety_node_present = 'safety_node' in node_names
            
            if was_present and not self.safety_node_present:
                self.get_logger().warn("Nie wykryto obecności węzła safety_node")
            elif not was_present and self.safety_node_present:
                self.get_logger().info("Węzeł safety_node został wykryty")
            
            self.last_node_check_time = now
        
        return self.safety_node_present
    
    def velocity_fresh(self) -> bool:
        if self.last_cmd_time is None:
            return False
        return (time.monotonic() - self.last_cmd_time) <= self.VELOCITY_TIMEOUT

    def send_velocity(self):
        
        allow_motion = self.check_network_deadman_node() and self.deadman_ok and self.velocity_fresh()

        if allow_motion:
            vx, vy, omega = self.vx, self.vy, self.omega
        else:
            vx, vy, omega = 0.0, 0.0, 0.0

        velocity_data = [vx, vy, omega]

        try:
            response = requests.post(
                URL + '/data/omnidrive',
                json=velocity_data,
                timeout=0.1
            )
            if response.status_code != 200:
                self.get_logger().error(
                    f'Błąd REST: {response.status_code}'
                )
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = VelocitySender()
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
