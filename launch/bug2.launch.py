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
    
    bug2 = Node(
        package='robotino_ros2',
        executable='bug2',
        name='bug2_node',
        output='screen',
        parameters=[{
            'obstacle_threshold': 0.6,    # minimalna odległość do przeszkody
            'odom_detect_distance': 0.3,  # próg detekcji przeszkód z odometrii
            'wall_distance': 0.2,         # preferowana odległość od ściany podczas wall-follow
            'wall_follow_side': 'left',   # wybrana strona podążania wzdłuż ściany left/right/auto
            'max_lin_vel': 0.2,           # maksymalna prędkość liniowa
            'max_ang_vel': 0.6,           # maksymalna prędkość kątowa
            'eps_goal': 0.1,              # tolerancja odległości do celu
            'eps_angle': 0.2              # tolerancja błędu orientacji
        }]
    )

    return LaunchDescription([
        lidar_launch,
        distance_sensors,
        bumper,
        odometry,
        robotino_tools,
        velocity_sender,
        bug2
    ])
