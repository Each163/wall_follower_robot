import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='two_wheeled_robot_mine', executable='robot_estimator.py',
            output='screen'),
        Node(package='two_wheeled_robot_mine', executable='robot_controller.py',
            output='screen'),
    ])