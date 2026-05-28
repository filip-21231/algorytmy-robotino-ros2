#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, ColorRGBA
from geometry_msgs.msg import PoseStamped, TransformStamped, Point, Pose2D
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

class RobotinoTools(Node):
    def __init__(self):
        super().__init__('robotino_tools')

        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value

        self.goal_sub = self.create_subscription(Pose2D, 'robotino/goal', self.goal_topic_callback, 10)

        self.sub = self.create_subscription(Float64MultiArray, 'robotino/odometry', self.odom_callback, 10)

        self.pose_pub = self.create_publisher(PoseStamped, 'robotino/pose', 10)
        self.path_pub = self.create_publisher(Path, 'robotino/path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'robotino/markers', 10)

        self.clear_service = self.create_service(Trigger, 'clear_path', self.clear_path_callback)

        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.tf_broadcaster = TransformBroadcaster(self)
        self.start_pose = None

        self.get_logger().info('Uruchomiono węzeł pomocniczy robotino_tools.')

    def goal_topic_callback(self, msg: Pose2D):
        self.goal_x = msg.x
        self.goal_y = msg.y

    def create_point_marker(self, id, x, y, color, label):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = label
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.05
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color = color
        return marker

    def create_delete_marker(self, id, label):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = label
        marker.id = id
        marker.action = Marker.DELETE
        return marker

    def clear_path_callback(self, request, response):
        self.path.poses = [] 
        self.start_pose = None
        
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value

        empty_path = Path()
        empty_path.header.stamp = self.get_clock().now().to_msg()
        empty_path.header.frame_id = 'odom'
        self.path_pub.publish(empty_path)

        del_markers = MarkerArray()
        del_markers.markers.append(self.create_delete_marker(0, "start"))
        del_markers.markers.append(self.create_delete_marker(1, "goal"))
        self.marker_pub.publish(del_markers)
        
        response.success = True
        response.message = "Ścieżka zresetowana."
        return response

    def odom_callback(self, msg):
        if len(msg.data) < 3:
            return
            
        x = msg.data[0]
        y = msg.data[1]
        omega = msg.data[2]

        if self.start_pose is None:
            self.start_pose = (x, y)

        qz = math.sin(omega / 2.0)
        qw = math.cos(omega / 2.0)

        now = self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = 'odom'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)
        self.pose_pub.publish(pose)

        self.path.header.stamp = now
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)

        markers = MarkerArray()

        if self.start_pose is not None:
            markers.markers.append(
                self.create_point_marker(0, self.start_pose[0], self.start_pose[1], 
                ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0), "start")
            )
        else:
            markers.markers.append(self.create_delete_marker(0, "start"))

        gx = self.goal_x if self.goal_x is not None else self.get_parameter('goal_x').value
        gy = self.goal_y if self.goal_y is not None else self.get_parameter('goal_y').value

        markers.markers.append(
            self.create_point_marker(1, gx, gy, 
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), "goal")
        )

        self.marker_pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = RobotinoTools()
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