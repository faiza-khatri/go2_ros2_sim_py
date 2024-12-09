#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
import tf_transformations
import tf2_ros
import math
from collections import deque

class DogOdometry(Node):
    def __init__(self):
        super().__init__('dog_odometry')

        # Объявление и получение параметров
        self.declare_parameter('verbose', False)
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        if self.verbose:
            self.get_logger().info(f"Verbose mode: {self.verbose}")

        self.declare_parameter('publish_rate', 50)
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().integer_value
        if self.verbose:
            self.get_logger().info(f"Publish rate: {publish_rate} Hz")

        self.declare_parameter('has_imu_heading', True)
        self.has_imu_heading = self.get_parameter('has_imu_heading').get_parameter_value().bool_value
        if self.verbose:
            self.get_logger().info(f"Has IMU heading: {self.has_imu_heading}")

        self.declare_parameter('enable_odom_tf', True)
        self.enable_odom_tf = self.get_parameter('enable_odom_tf').get_parameter_value().bool_value
        if self.verbose:
            self.get_logger().info(f"Enable odom TF: {self.enable_odom_tf}")

        self.declare_parameter('base_frame_id', 'base_footprint')
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        if self.verbose:
            self.get_logger().info(f"Base frame ID: {self.base_frame_id}")

        self.declare_parameter('odom_frame_id', 'odom')
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        if self.verbose:
            self.get_logger().info(f"Odom frame ID: {self.odom_frame_id}")

        # Инициализация переменных одометрии
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0  # Будет обновляться из IMU

        # Инициализация времени
        self.last_position_time = self.get_clock().now()

        # Публикация одометрии
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # Подписка на IMU
        if self.has_imu_heading:
            self.imu_sub = self.create_subscription(
                Imu,
                '/robot1/imu_plugin/out',
                self.imu_callback,
                10
            )

        # Подписка на /controller_velocity
        self.controller_velocity_sub = self.create_subscription(
            Twist,
            '/controller_velocity',
            self.controller_velocity_callback,
            10
        )

        # Транслятор TF
        if self.enable_odom_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Таймер для публикации одометрии
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Dog Odometry Node has been started.")

        # Ограничения на скорости
        self.MAX_LINEAR_VELOCITY = 1.0  # Максимальная линейная скорость в м/с
        self.MAX_ANGULAR_VELOCITY = 2.0  # Максимальная угловая скорость в рад/с

        # Инициализация переменных для хранения угловой скорости из IMU
        self.imu_angular_velocity = 0.0

    def controller_velocity_callback(self, msg):
        # Используется только linear.x
        self.linear_velocity = msg.linear.x
        if self.verbose:
            self.get_logger().info(f"Controller Linear Velocity: {self.linear_velocity:.6f} m/s")

    def imu_callback(self, msg):
        # Извлечение ориентации из данных IMU
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(orientation_list)

        self.theta = yaw

        # Извлечение угловой скорости из IMU
        self.imu_angular_velocity = msg.angular_velocity.z

        if self.verbose:
            self.get_logger().info(f"IMU Yaw: {self.theta:.6f} rad")
            self.get_logger().info(f"IMU Angular Velocity: {self.imu_angular_velocity:.6f} rad/s")

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_position_time).nanoseconds / 1e9
        if dt <= 0.0:
            return  # Избегаем деления на ноль или отрицательных dt

        # Обновление положения на основе linear_velocity
        delta_x = self.linear_velocity * math.cos(self.theta) * dt
        delta_y = self.linear_velocity * math.sin(self.theta) * dt
        self.x += delta_x
        self.y += delta_y

        # Создание сообщения одометрии
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        quaternion = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )

        # Установка скоростей
        odom.twist.twist.linear.x = self.linear_velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.imu_angular_velocity

        # Публикация одометрии
        self.odom_pub.publish(odom)

        # Публикация TF
        if self.enable_odom_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id

            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = Quaternion(
                x=quaternion[0],
                y=quaternion[1],
                z=quaternion[2],
                w=quaternion[3]
            )

            self.tf_broadcaster.sendTransform(t)

        # Обновление времени
        self.last_position_time = current_time

        if self.verbose:
            self.get_logger().info(f"Position Updated: x={self.x:.6f} m, y={self.y:.6f} m, theta={self.theta:.6f} rad")
            self.get_logger().info(f"Delta_x: {delta_x:.6f} m, Delta_y: {delta_y:.6f} m")
            self.get_logger().info(f"Linear Velocity from Controller: {self.linear_velocity:.6f} m/s")

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DogOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
