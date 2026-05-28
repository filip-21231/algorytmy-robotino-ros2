#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ldlidar_ros2'),
                'launch',
                'ld14p.launch.py'
            )
        )
    )

    distance_sensors = Node(
        package='robotino_ros2',
        executable='distance_sensors',
        name='distance_sensors',
        output='screen',
    )

    bumper = Node(
        package='robotino_ros2',
        executable='bumper',
        name='bumper',
        output='screen',
    )

    odometry = Node(
        package='robotino_ros2',
        executable='odometry_robotino',
        name='odometry',
        output='screen',
    )

    robotino_tools = Node (
        package='robotino_ros2',
        executable='robotino_tools',
        name='robotino_tools',
        output='screen',
    )

    velocity_sender = Node(
        package='robotino_ros2',
        executable='velocity_sender',
        name='velocity_sender',
        output='screen',
    )
    

    apf = Node(
        package='robotino_ros2',
        executable='apf',
        name='apf_node',
        output='screen',
        parameters=[{
            'k_att': 0.6,             # wzmocnienie siły przyciągającej
            'kr_ir': 2.0,             # wzmocnienie siły odpychającej IR
            'kr_lidar': 0.08,          # wzmocnienie siły odpychającej LiDAR
            'rep_range_ir': 0.2,     # zasięg oddziaływania czujników IR
            'rep_range_lidar': 0.6,   # zasięg oddziaływania LiDAR
            'max_lin_vel': 0.2,      # maksymalna prędkość liniowa robota
            'max_ang_vel': 0.6,       # maksymalna prędkość kątowa robota
            'k_theta': 1.2,           # wzmocnienie regulatora orientacji
            'eps_goal': 0.1,          # tolerancja odległości do celu
            'eps_angle': 0.2,         # tolerancja błędu orientacji
        }]
    )

    return LaunchDescription([
        lidar_launch,
        distance_sensors,
        bumper,
        odometry,
        robotino_tools,
        velocity_sender,
        apf
    ])
