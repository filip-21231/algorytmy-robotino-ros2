#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import requests
import json

URL = 'http://192.168.0.12'

class OdometryPub(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'robotino/odometry', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Uruchomiono węzeł przesyłający dane o położeniu robotino')
    def timer_callback(self):
        try:
            response = requests.get(URL + "/data/odometry")

            if response.status_code == 200:
                data = response.json()

                try:
                    x = float(data[0])
                    y = float(data[1])
                    rot = float(data[2])
                    vx = float(data[3])
                    vy = float(data[4])
                    omega = float(data[5])
                    seq = float(data[6])

                    odometry_data = Float64MultiArray()
                    odometry_data.data = [x, y, rot, vx, vy, omega, seq]

                    self.publisher_.publish(odometry_data)

                except (ValueError, IndexError) as e:
                    self.get_logger().error(f"Błąd przetwarzania danych odometrii: {e}")
            else:
                self.get_logger().error(f"Nie udało się pobrać danych: {response.status_code}")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Błąd podczas łączenia z serwerem: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Zatrzymywanie węzła...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
