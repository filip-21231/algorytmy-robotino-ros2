#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import requests
import time
import threading
from concurrent.futures import ThreadPoolExecutor

URL = 'http://192.168.0.12'

class DistancePub(Node):
    def __init__(self):
        super().__init__('distance_sensors_publisher')

        self.publisher_ = self.create_publisher(Float64MultiArray, 'robotino/distance_sensors', 10)

        self.timer = self.create_timer(0.02, self.timer_callback)

        self.min_val = 0.04
        self.max_val = 0.30
        self.safety_distance = 0.18

        self.alpha_approach = 1.0
        self.alpha_retreat = 0.85 

        self.values = [self.max_val for _ in range(9)]

        self.latest_data = [self.max_val for _ in range(9)]
        self.last_update_time = time.time()
        self.data_lock = threading.Lock()

        self.session = requests.Session()
        self.session.headers.update({'Connection': 'keep-alive'})

        self.thread_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='http_fetch')
        self.fetch_running = True
        self.fetch_future = self.thread_executor.submit(self.fetch_loop)

        self.stale_count = 0
        self.fetch_errors = 0

        self.get_logger().info('Uruchomiono węzeł przesyłający dane z czujników IR (50 Hz, async)')

    def fetch_loop(self):
        consecutive_errors = 0
        
        while self.fetch_running and rclpy.ok():
            try:
                response = self.session.get(
                    URL + '/data/distancesensorarray',
                    timeout=(0.1, 0.2)
                )

                if response.status_code == 200:
                    data = response.json()

                    if isinstance(data, list) and len(data) == 9:
                        with self.data_lock:
                            self.latest_data = [float(x) for x in data]
                            self.last_update_time = time.time()
                        
                        consecutive_errors = 0 
                    else:
                        self.get_logger().warn(
                            f"Nieprawidłowy format danych: type={type(data)}, len={len(data) if isinstance(data, list) else 'N/A'}"
                        )
                        consecutive_errors += 1
                else:
                    self.get_logger().warn(f"Błąd HTTP: {response.status_code}")
                    consecutive_errors += 1

            except requests.exceptions.Timeout:
                consecutive_errors += 1
                if consecutive_errors % 10 == 1:
                    self.get_logger().warn(f"Timeout czujników IR (#{consecutive_errors})")
                    
            except requests.exceptions.RequestException as e:
                consecutive_errors += 1
                if consecutive_errors % 10 == 1:
                    self.get_logger().warn(f"Błąd połączenia: {e}")
            
            except Exception as e:
                self.get_logger().error(f"Nieoczekiwany błąd w fetch_loop: {e}")
                consecutive_errors += 1

            time.sleep(0.01)

    def filter_ir(self, i, new):
        val = self.values[i]

        new = max(self.min_val, min(self.max_val, new))

        if new < self.safety_distance:
            self.values[i] = new
            return new

        if new < val:
            val = new

        else:
            val = self.alpha_retreat * new + (1 - self.alpha_retreat) * val

        self.values[i] = val
        return val

    def timer_callback(self):
        with self.data_lock:
            data = self.latest_data.copy()
            data_age = time.time() - self.last_update_time

        stale = False
        if data_age > 0.2:
            stale = True
            self.stale_count += 1
            if self.stale_count % 20 == 1:
                self.get_logger().warn(f"Dane czujników są przestarzałe ({data_age:.3f}s)")

        filtered_values = []

        for i in range(9):
            try:
                raw = float(data[i])
                filtered = self.filter_ir(i, raw)
                filtered_values.append(filtered)

            except (ValueError, IndexError, TypeError) as e:
                self.get_logger().warn(f"Błąd danych czujnika {i}: {e}")
                filtered_values.append(self.values[i])

        msg = Float64MultiArray()
        msg.data = filtered_values
        self.publisher_.publish(msg)

    def shutdown(self):
        self.get_logger().info('Zamykanie węzła...')
        
        self.fetch_running = False
        
        try:
            self.fetch_future.result(timeout=1.0)
        except Exception:
            pass

        self.thread_executor.shutdown(wait=False)
        
        try:
            self.session.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = DistancePub()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Zatrzymanie węzła...')
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
