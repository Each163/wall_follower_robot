#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray

class Estimator(Node):
    def __init__(self):
        super().__init__('Estimator')
        self.subscriber_odom = self.create_subscription(
            Odometry,
            '/demo/odom',
            self.odom_callback,
            10
        )

        # Publish state estimation
        # format: (x, y, yaw)
        self.publisher_state_est = self.create_publisher(
            Float64MultiArray,
            '/demo/state_est',
            10
        )

    def odom_callback(self, msg: Odometry):
        roll, pitch, yaw = self.euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        obs_state_vector_x_y_yaw = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
        self.publish_estimated_state(obs_state_vector_x_y_yaw)

    def euler_from_quaternion(self, x, y, z, w):
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = 2.0 * (w * y - z * x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def publish_estimated_state(self, state_vector_x_y_yaw):
        msg = Float64MultiArray()
        msg.data = state_vector_x_y_yaw
        self.publisher_state_est.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    estimator = Estimator()
    rclpy.spin(estimator)
    estimator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()