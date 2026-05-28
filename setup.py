#!/usr/bin/env python3
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robotino_ros2'

setup(
    name=package_name,                  
    version='1.0.0',                  
    packages=find_packages(exclude=['test']),  

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],

    install_requires=['setuptools','requests','json','math','numpy', 'os','time', 'subprocess', 'sys','signal','select','termios','tty'],
    zip_safe=True,                     
    maintainer='filip-21231',                  
    maintainer_email='',  
    description='Pakiet ROS 2 zawierający mostek REST Robotino oraz algorytmy sterowania Bug2 i APF',  
    license='Apache-2.0',               
    tests_require=['pytest'],           

    entry_points={
        'console_scripts': [
            'odometry_robotino = robotino_ros2.odometry_robotino:main',
            'bumper = robotino_ros2.bumper:main',
            'distance_sensors = robotino_ros2.distance_sensors:main',
            'terminal_control = robotino_ros2.terminal_control:main',
            'velocity_sender = robotino_ros2.velocity_sender:main',
            'bug2 = robotino_ros2.bug2:main',
            'robotino_tools = robotino_ros2.robotino_tools:main',
            'apf = robotino_ros2.apf:main',
            'safety_node = robotino_ros2.safety_node:main'
        ],
    },
)
