#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class GroundFilter(Node):
    def __init__(self):
        super().__init__('ground_filter')
        self.declare_parameter('input_topic', '/robot1/points/points_timed')
        self.declare_parameter('output_topic', '/livox/points_filtered')
        self.declare_parameter('ground_z_thresh', 0.15)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.z_thresh = self.get_parameter('ground_z_thresh').value

        self.pub = self.create_publisher(PointCloud2, out_topic, 10)
        self.sub = self.create_subscription(PointCloud2, in_topic, self.cb, 10)

    def cb(self, msg):
        points = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        if points.shape[0] == 0:
            return
        mask = points[:, 2] > self.z_thresh
        filtered = points[mask]
        cloud = pc2.create_cloud_xyz32(msg.header, filtered.tolist())
        self.pub.publish(cloud)

def main():
    rclpy.init()
    node = GroundFilter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
