import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import ros2_numpy as rnp

from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R


import numpy as np

class LidarDummyNode(Node):

    def __init__(self, rotation_rate=10.0):
        super().__init__('lidar_fusion_node')

        self.lidar_front_pub = self.create_publisher(
            msg_type = PointCloud2,
            topic = "lidar_front",
            qos_profile = 10
        )

        self.lidar_rear_pub = self.create_publisher(
            msg_type = PointCloud2,
            topic = "lidar_rear",
            qos_profile = 10
        )
        
        self.br = TransformBroadcaster(self)


        self.timer = self.create_timer(1/rotation_rate, self.publish_lidar)


    def publish_lidar(self):

        # make bumps
        x1, y1 = np.meshgrid(np.linspace(-5, 0, 100), np.linspace(-5, 5, 100))
        z1 = np.sin(x1*5) * np.cos(y1*5) / 5 
        lidar_front_pcd = np.stack([x1,y1,z1], axis=-1).astype(np.float32)
        lidar_front_pcd.dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32)]

        x2, y2 = np.meshgrid(np.linspace(0, 5, 100), np.linspace(-5, 5, 100))
        z2 = np.sin(x2*5) * np.cos(y2*5) / 5 
        lidar_rear_pcd = np.stack([x2,y2,z2], axis=-1).astype(np.float32)
        lidar_rear_pcd.dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32)]

        t = TransformStamped()
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        q = R.from_euler('z', 0, degrees=True).as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        t.child_frame_id = 'base_link'
        now = self.get_clock().now().to_msg()
        t.header.stamp = now

        lidar_front_msg = rnp.msgify(PointCloud2, lidar_front_pcd)
        lidar_front_msg.header.frame_id = 'lidar_front'
        lidar_front_msg.header.stamp = now
        self.lidar_front_pub.publish(lidar_front_msg)

        t.header.frame_id = 'lidar_front'
        self.br.sendTransform(t)

        lidar_rear_msg = rnp.msgify(PointCloud2, lidar_rear_pcd)
        lidar_rear_msg.header.frame_id = 'lidar_rear'
        lidar_rear_msg.header.stamp = now
        self.lidar_rear_pub.publish(lidar_rear_msg)

        t.header.frame_id = 'lidar_rear'
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)


    lidar_dummy_node = LidarDummyNode(rotation_rate=10)

    rclpy.spin(lidar_dummy_node)

    lidar_dummy_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()