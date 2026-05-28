#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import requests

URL = 'http://192.168.0.12'

class BumperPub(Node):
    def __init__(self):
        super().__init__('bumper_publisher')

        self.publisher_ = self.create_publisher(Bool, 'robotino/bumper', 10)
        
        timer_period = 0.1

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        try:
            response = requests.get(URL + '/data/bumper')

            if response.status_code == 200:
                try:
                    bumper_value = response.json() 
                    bumper_msg = bumper_value.get('value')
                    
                    if bumper_msg in [True, False]:
                        bumper = Bool()
                        bumper.data = bumper_msg

                        self.publisher_.publish(bumper)
                    else:
                        self.get_logger().error(f'Otrzymano nieprawidłową wartość ze zderzaka: {bumper_msg}')
                except ValueError as e:
                    self.get_logger().error(f'Błąd konwersji danych: {e}')
            else:
                self.get_logger().error(f'Nie udało się pobrać danych: {response.status_code}, {response.text}')
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'Błąd podczas łączenia z serwerem: {e}')

def main(args=None):
    rclpy.init(args=args)

    node = BumperPub()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Zatrzymywanie węzła...')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
