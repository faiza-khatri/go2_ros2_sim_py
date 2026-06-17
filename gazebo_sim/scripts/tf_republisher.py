#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

class TFRepublisher(Node):
    def __init__(self):
        super().__init__('tf_republisher')
        
        tf_qos = QoSProfile(depth=10)
        static_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.pub_tf = self.create_publisher(TFMessage, '/tf', tf_qos)
        self.pub_tf_static = self.create_publisher(TFMessage, '/tf_static', static_qos)
        self.sub_tf = self.create_subscription(TFMessage, '/robot1/tf', self.tf_cb, tf_qos)
        self.sub_tf_static = self.create_subscription(TFMessage, '/robot1/tf_static', self.tf_static_cb, static_qos)
        self.get_logger().info('TF Republisher started')

    def tf_cb(self, msg):
        self.pub_tf.publish(msg)

    def tf_static_cb(self, msg):
        self.pub_tf_static.publish(msg)

def main():
    rclpy.init()
    node = TFRepublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
