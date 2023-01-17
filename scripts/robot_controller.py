#!/usr/bin/env python3

import math
import rclpy

# Enables pauses in the execution of code
from time import sleep 
 
# Used to create nodes
from rclpy.node import Node
 
# Enables the use of the string message type
from std_msgs.msg import String 
 
# Twist is linear and angular velocity
from geometry_msgs.msg import Twist     
                     
# Handles LaserScan messages to sense distance to obstacles (i.e. walls)        
from sensor_msgs.msg import LaserScan    
 
# Handle Pose messages
from geometry_msgs.msg import Pose 
 
# Handle float64 arrays
from std_msgs.msg import Float64MultiArray
                     
# Handles quality of service for LaserScan data
from rclpy.qos import qos_profile_sensor_data 
 
# Scientific computing library
import numpy as np 

class Controller(Node):
    def __init__(self):
        super().__init__('Controller')

        # Data format: (x, y, yaw)
        self.subscriber_state = self.create_subscription(
            Float64MultiArray,
            '/demo/state_est',
            self.state_estimate_callback,
            10
        )
        self.subscriber_state   # prevent unused variable warning

        self.subscriber_scan = self.create_subscription(
            LaserScan,
            'demo/laser/out',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data
        )

        self.publisher_wheels = self.create_publisher(
            Twist,
            '/demo/cmd_vel',
            10
        )

        # Initialize the boundary of the LaserScan sensor data 
        # Values are in meters
        self.left_dist = 99999.9
        self.leftfront_dist = 99999.9
        self.front_dist = 99999.9
        self.rightfront_dist = 99999.9
        self.right_dist = 99999.9

        # Maximum forward speed of the robot in meters per second
        self.forward_speed = 0.1
        self.forward_speed_slow = 0.015

        # Current position and orientation of the robot in the global
        # reference frame
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Finite states for the robot
        # "left_turn"
        # "right_turn"
        # "forward"
        # "stop"
        self.cmd = "left_turn"

        # Turning speed in rad/s
        self.turning_speed_fast = 0.1
        self.turning_speed_slow = 0.05

        # Keep the distance from wall (meter)
        self.dist_from_wall_upper_bound = 0.5
        self.dist_from_wall_lower_bound = 0.2

        # Whether a wall was found
        self.found_wall_flag = False

    def state_estimate_callback(self, msg):
        curr_state = msg.data
        self.current_x = curr_state[0]
        self.current_y = curr_state[1]
        self.current_yaw = curr_state[2]
        # self.get_logger().info(f"x:{self.current_x}, y:{self.current_y}, yaw:{self.current_yaw}\n")

        # the frequency of this function is higher than the scan_callback(),
        # so the control function is put here
        self.follow_wall()

    def scan_callback(self, msg: LaserScan):
        self.left_dist = np.min(msg.ranges[158: 180]) 
        self.leftfront_dist = np.min(msg.ranges[113: 157])       # [135-22.5, 135+22.5]
        self.front_dist = np.min(msg.ranges[68: 112])
        self.rightfront_dist = np.min(msg.ranges[23: 67])
        self.right_dist = np.min(msg.ranges[0: 22])

        self.get_logger().info(f'''left:{self.left_dist}; left_front:{self.leftfront_dist};
                                front:{self.front_dist}; right_front:{self.rightfront_dist};
                                right:{self.right_dist}''')

    def follow_wall(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0    

        d = self.dist_from_wall_upper_bound

        # avoid hitting a wall
        if self.front_dist < self.dist_from_wall_lower_bound:
            self.cmd = "go back"
            msg.linear.x = -self.forward_speed
        # follow the wall on the right side
        elif self.rightfront_dist < d:
            self.found_wall_flag = True
            if self.front_dist < d:
                self.cmd = "turn left"
                msg.angular.z = self.turning_speed_fast
            elif self.rightfront_dist < self.dist_from_wall_lower_bound:
                self.cmd = "move and turn left"
                msg.linear.x = self.forward_speed_slow
                msg.angular.z = self.turning_speed_slow
            else:
                self.cmd = "follow wall"
                msg.linear.x = self.forward_speed_slow
        elif self.front_dist < d or self.leftfront_dist < d:
            self.cmd = "turn left"
            msg.angular.z = self.turning_speed_fast
        else:
            if not self.found_wall_flag:
                self.cmd = "search for wall"
                msg.linear.x = self.forward_speed_slow
            else:
                self.cmd = "turn right and follow wall"
                msg.linear.x = self.forward_speed_slow
                msg.angular.z = -self.turning_speed_slow
                if self.right_dist > d:
                    self.found_wall_flag = False

        # # follow the wall on the right side
        # if self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist > d:
        #     self.cmd = "search for wall on the right"
        #     msg.linear.x = self.forward_speed_slow
        #     msg.angular.z = -self.turning_speed_slow    # turn right to find a wall
        # elif self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist < d:
        #     if self.rightfront_dist < self.dist_from_wall_lower_bound:
        #         self.cmd = "turn left"
        #         msg.linear.x = self.forward_speed_slow
        #         msg.angular.z = self.turning_speed_fast
        #     else:
        #         self.cmd = "follow wall"
        #         msg.linear.x = self.forward_speed_slow
        # # deal with other conditions
        # elif self.leftfront_dist > d and self.front_dist < d and self.rightfront_dist > d:
        #     self.cmd = "turn left"
        #     msg.angular.z = self.turning_speed_fast     # let the right side of the robot face walls
        # elif self.leftfront_dist > d and self.front_dist < d and self.rightfront_dist < d:
        #     self.cmd = "turn left"
        #     msg.angular.z = self.turning_speed_fast 
        # elif self.leftfront_dist < d and self.front_dist > d and self.rightfront_dist > d:
        #     self.cmd = "search for wall on the right"
        #     msg.linear.x = self.forward_speed_slow
        #     msg.angular.z = -self.turning_speed_slow    # turn right to find a wall
        # elif self.leftfront_dist < d and self.front_dist > d and self.rightfront_dist < d:
        #     self.cmd = "follow wall"
        #     msg.linear.x = self.forward_speed_slow
        #     # msg.angular.z = -self.turning_speed_slow    # turn right to find a wall
        # elif self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist > d:
        #     self.cmd = "turn left"
        #     msg.angular.z = self.turning_speed_fast 
        # elif self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist < d:
        #     self.cmd = "turn left"
        #     msg.angular.z = self.turning_speed_fast 
        # else:
        #     pass

        self.get_logger().info(f"control cmd: {self.cmd}")

        self.publisher_wheels.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        