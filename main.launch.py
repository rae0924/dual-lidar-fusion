from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_fusion',
            executable='lidar_fusion_node',
            name='lidar_fusion_node'
        ),
        Node(
            package='lidar_dummy',
            executable='lidar_dummy_node',
            name='lidar_dummy_node'
        )
    ])