#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
import tf_transformations
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class QuadrupedOdometryNode(Node):
    def __init__(self, node_name="quadruped_odometry"):
        super().__init__(node_name)

        # Объявление параметров без use_sim_time
        self.declare_parameter("verbose", True)
        self.declare_parameter("publish_rate", 50)
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("imu_topic", "/imu_plugin/out")
        self.declare_parameter("base_frame_id", "base_footprint")
        self.declare_parameter("velocity_topic", "/controller_velocity")  # Новый параметр

        # Получение значений параметров
        verbose = self.get_parameter("verbose").value
        publish_rate = self.get_parameter("publish_rate").value
        imu_topic = self.get_parameter("imu_topic").value
        velocity_topic = self.get_parameter("velocity_topic").value

        # Получение значения use_sim_time из основного узла через параметры или пространство имен
        use_sim_time = self.get_parameter("use_sim_time").get_parameter_value().bool_value

        if verbose:
            self.get_logger().info(f"Publishing rate: {publish_rate}")
            self.get_logger().info(f"Velocity topic: {velocity_topic}")
            self.get_logger().info(f"IMU topic: {imu_topic}")

        # Переменные для одометрии
        self.x = 0.0
        self.y = 0.0
        self.prev_time = None

        # Скорости из контроллера
        self.velocity_x = 0.0
        self.velocity_y = 0.0

        # Настройка QoS для подписки на IMU
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE  # Или QoSReliabilityPolicy.RELIABLE

        self.velocity_sub = self.create_subscription(Twist, velocity_topic, self.velocity_callback, 10)
        self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_callback, qos_profile)

        # Публикация одометрии
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_imu_quat = [0.0, 0.0, 0.0, 1.0]  # Инициализируем единичным кватернионом
        self.imu_received = False  # Флаг получения данных IMU

        # Таймер для публикации одометрии
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_odom)

    def imu_callback(self, msg):
        self.get_logger().info("IMU callback triggered")
        try:
            self.last_imu_quat = [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ]
            self.imu_received = True
            self.get_logger().info(f"Received IMU Quaternion: {self.last_imu_quat}")
        except Exception as e:
            self.get_logger().error(f"Error in imu_callback: {e}")

    def velocity_callback(self, msg):
        """
        Обновление текущих команд скоростей из топика.
        Используем только линейные скорости.
        """
        self.velocity_x = msg.linear.x
        self.velocity_y = msg.linear.y
        # self.angular_velocity = msg.angular.z  # Игнорируем угловую скорость из контроллера
        if self.get_parameter("verbose").value:
            self.get_logger().debug(f"Received Velocity: linear.x={self.velocity_x}, linear.y={self.velocity_y}")

    def publish_odom(self):
        """
        Публикация одометрии, используя данные скорости и ориентации из IMU.
        """
        # Интервал времени
        current_time_msg = self.get_clock().now().to_msg()
        current_time = current_time_msg.sec + current_time_msg.nanosec * 1e-9
        if self.prev_time is None:
            self.prev_time = current_time
            return

        dt = current_time - self.prev_time
        self.prev_time = current_time

        if dt <= 0.0:
            self.get_logger().warn(f"Non-positive dt: {dt}")
            return

        # Обновление позиций на основе линейных скоростей без учёта ориентации
        self.x += self.velocity_x * dt
        self.y += self.velocity_y * dt

        # Проверка, получены ли данные IMU
        if not self.imu_received:
            self.get_logger().warn("IMU data not received yet. Using default orientation.")
            quat = tf_transformations.quaternion_from_euler(0, 0, 0)
        else:
            quat = self.last_imu_quat

        # Публикация одометрии
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time_msg
        odom_msg.header.frame_id = self.get_parameter("odom_frame_id").value
        odom_msg.child_frame_id = self.get_parameter("base_frame_id").value
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0  # Предполагаем движение по плоскости

        # Используем ориентацию из IMU
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # Публикуем линейные скорости
        odom_msg.twist.twist.linear.x = self.velocity_x
        odom_msg.twist.twist.linear.y = self.velocity_y

        # Угловая скорость устанавливается в 0, так как используем данные IMU
        odom_msg.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom_msg)

        # Публикация TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = odom_msg.header.stamp
        tf_msg.header.frame_id = odom_msg.header.frame_id
        tf_msg.child_frame_id = odom_msg.child_frame_id
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    node = QuadrupedOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
