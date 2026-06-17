#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np

class PointCloudStamper(Node):
    def __init__(self):
        super().__init__('pointcloud_stamper')
        self.sub = self.create_subscription(
            PointCloud2, '/robot1/points/points',
            self.callback, 10)
        self.pub = self.create_publisher(
            PointCloud2, '/robot1/points/points_timed', 10)
        self.get_logger().info('PointCloud stamper ready')

    def callback(self, msg):
        n_points = msg.width * msg.height
        if n_points == 0:
            return

        new_fields = list(msg.fields) + [
            PointField(name='time', offset=msg.point_step,
                      datatype=PointField.FLOAT32, count=1)
        ]

        new_point_step = msg.point_step + 4
        old_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            n_points, msg.point_step)

        # Relative timestamps in microseconds across 100ms scan period
        scan_period_us = 100000.0
        time_offsets = np.linspace(0, scan_period_us, n_points,
                                   dtype=np.float32)
        time_bytes = time_offsets.view(np.uint8).reshape(n_points, 4)

        new_data = np.hstack([old_data, time_bytes])

        out = PointCloud2()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.fields = new_fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = new_point_step
        out.row_step = new_point_step * msg.width
        out.is_dense = msg.is_dense
        out.data = new_data.tobytes()
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(PointCloudStamper())

if __name__ == '__main__':
    main()