from launch import LaunchDescription
from launch_ros.actions import Node

from os import path

def generate_launch_description():

    launch_path = path.realpath(__file__)
    launch_dir = path.dirname(launch_path)

    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[path.join(launch_dir, 'voltron.urdf')]
    )
    
    lidar_fusion = Node(
        package='lidar_fusion',
        executable='lidar_fusion_node',
        name='lidar_fusion_node',
        remappings=[
            ('/lidar_front/velodyne_points', '/lidar_front'),
            ('/lidar_rear/velodyne_points', '/lidar_rear'),
        ]
    )

    lidar_dummy = Node(
        package='lidar_dummy',
        executable='lidar_dummy_node',
        name='lidar_dummy_node'
    )

    return LaunchDescription([
        urdf_publisher,
        lidar_fusion,
        lidar_dummy
    ])