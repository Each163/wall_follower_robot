#!/usr/bin/env python3

import os
import sys
import rclpy

from ament_index_python import get_package_share_directory

from gazebo_msgs.srv import SpawnEntity

def main():
    # Get the input from user
    argv = sys.argv[1:]

    # Start node
    rclpy.init()

    # sdf_file_path = os.path.join(
    #     get_package_share_directory('two_wheeled_robot_mine'), 'sdf', 'two_wheeled_robot.sdf'
    # )
    sdf_file_path = os.path.join(
        get_package_share_directory('wall_follower_robot'), 'sdf', 'model.sdf'
    )
    print(f"robot_sdf_path={sdf_file_path}")

    # Create node and client for the gazebo service "/spawn_entity"
    node = rclpy.create_node('entity_spawner')
    node.get_logger().info('Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    # Connect to the service
    node.get_logger().info('Connecting to `/spawn_entity` service...')
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info('...connected!')

    # Set the data for request
    request = SpawnEntity.Request()
    request.name = argv[0]
    request.xml = open(sdf_file_path, 'r').read()
    request.robot_namespace = argv[1]
    request.initial_pose.position.x = float(argv[2])
    request.initial_pose.position.y = float(argv[3])
    request.initial_pose.position.z = float(argv[4])

    # Send request
    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception()
        )
    
    node.get_logger().info('Done! Shutting down node.')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()