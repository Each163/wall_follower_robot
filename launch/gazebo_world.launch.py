import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('two_wheeled_robot_mine')
    world_file_name = 'warehouse.world'
    world = os.path.join(pkg_dir, 'worlds', world_file_name)

    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_dir, 'sdf')
    os.environ["TURTLEBOT3_MODEL"] = os.path.join(pkg_dir, 'sdf')

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'], 
        output='screen'
    )

    spawn_entity = Node(
        package='two_wheeled_robot_mine',
        executable='spawn_robot.py',
        arguments=['WarehouseBot', 'demo', '-1.5', '-4.0', '0.0'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
    ])