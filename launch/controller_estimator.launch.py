import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='wall_follower_robot', executable='robot_estimator.py',
            output='screen'),
        Node(package='wall_follower_robot', executable='robot_controller.py',
            output='screen'),
    ])