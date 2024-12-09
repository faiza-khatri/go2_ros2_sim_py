#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
import tf_transformations
import tf2_ros
import math
from collections import deque
import numpy as np
from scipy.signal import medfilt

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

        self.declare_parameter('open_loop', False)
        self.open_loop = self.get_parameter('open_loop').get_parameter_value().bool_value
        if self.verbose:
            self.get_logger().info(f"Open loop: {self.open_loop}")

        self.declare_parameter('has_imu_heading', True)
        self.has_imu_heading = self.get_parameter('has_imu_heading').get_parameter_value().bool_value
        if self.verbose:
            self.get_logger().info(f"Has IMU heading: {self.has_imu_heading}")

        self.declare_parameter('is_gazebo', False)
        self.is_gazebo = self.get_parameter('is_gazebo').get_parameter_value().bool_value
        if self.verbose:
            self.get_logger().info(f"Is Gazebo: {self.is_gazebo}")

        self.declare_parameter('thigh_length', 0.2)  # Примерное значение в метрах
        thigh_length = self.get_parameter('thigh_length').get_parameter_value().double_value
        if self.verbose:
            self.get_logger().info(f"Thigh length: {thigh_length} m")

        self.declare_parameter('base_width', 0.3)  # Примерное значение в метрах
        base_width = self.get_parameter('base_width').get_parameter_value().double_value
        if self.verbose:
            self.get_logger().info(f"Base width: {base_width} m")

        self.declare_parameter('encoder_resolution', 150)
        encoder_resolution = self.get_parameter('encoder_resolution').get_parameter_value().integer_value
        if self.verbose:
            self.get_logger().info(f"Encoder resolution: {encoder_resolution}")

        self.declare_parameter('steer_pos_multiplier', 1.0)
        self.steer_pos_multiplier = self.get_parameter('steer_pos_multiplier').get_parameter_value().double_value
        if self.verbose:
            self.get_logger().info(f"Steer position multiplier: {self.steer_pos_multiplier}")

        self.declare_parameter('velocity_rolling_window_size', 50)  # Увеличено для лучшего сглаживания
        velocity_rolling_window_size = self.get_parameter('velocity_rolling_window_size').get_parameter_value().integer_value
        if self.verbose:
            self.get_logger().info(f"Velocity rolling window size: {velocity_rolling_window_size}")

        self.declare_parameter('base_frame_id', 'base_footprint')
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        if self.verbose:
            self.get_logger().info(f"Base frame ID: {self.base_frame_id}")

        self.declare_parameter('imu_topic', '/imu_plugin/out')
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        if self.verbose:
            self.get_logger().info(f"IMU Topic: {self.imu_topic}")

        self.declare_parameter('odom_frame_id', 'odom')
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        if self.verbose:
            self.get_logger().info(f"Odom frame ID: {self.odom_frame_id}")

        self.declare_parameter('enable_odom_tf', True)
        self.enable_odom_tf = self.get_parameter('enable_odom_tf').get_parameter_value().bool_value
        if self.verbose:
            self.get_logger().info(f"Enable odom TF: {self.enable_odom_tf}")

        self.declare_parameter('clock_topic', 'clock')
        self.clock_topic = self.get_parameter('clock_topic').get_parameter_value().string_value
        if self.verbose:
            self.get_logger().info(f"Clock Topic: {self.clock_topic}")

        # Имена суставов квадропеда
        self.joint_names = [
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint'
        ]

        # Инициализация позиций суставов и предыдущих позиций
        self.joint_positions = {name: 0.0 for name in self.joint_names}
        self.prev_joint_positions = {name: 0.0 for name in self.joint_names}

        # Инициализация переменных одометрии
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0  # Будет обновляться из IMU

        # Инициализация отдельных временных меток
        self.last_joint_time = self.get_clock().now()
        self.last_position_time = self.get_clock().now()

        # Сохранение параметров ног
        self.thigh_length = thigh_length
        self.base_width = base_width
        self.encoder_resolution = encoder_resolution

        # Логирование параметров ног
        if self.verbose:
            self.get_logger().info(f"Thigh length: {self.thigh_length} m")
            self.get_logger().info(f"Base width: {self.base_width} m")

        # Инициализация паблишеров и подписчиков
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Подписка на IMU (если необходимо)
        if self.has_imu_heading:
            self.imu_sub = self.create_subscription(
                Imu,
                self.imu_topic,
                self.imu_callback,
                10
            )
        else:
            # Обработка альтернативных источников направления, если необходимо
            pass

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

        # Порог для фильтрации малых изменений позиций
        self.position_threshold = 1e-4  # Увеличен для лучшей фильтрации

        # Инициализация истории скоростей для фильтрации
        self.linear_velocity_history = deque(maxlen=velocity_rolling_window_size)
        self.angular_velocity_history = deque(maxlen=velocity_rolling_window_size)

        # Инициализация буферов для медианного фильтра
        self.left_thigh_velocity_buffer = []
        self.right_thigh_velocity_buffer = []

        # Ограничения на скорости
        self.MAX_LINEAR_VELOCITY = 1.0  # Максимальная линейная скорость в м/с
        self.MAX_ANGULAR_VELOCITY = 2.0  # Максимальная угловая скорость в рад/с

        # Инициализация переменных для хранения угловой скорости из IMU
        self.imu_angular_velocity = 0.0

    def joint_state_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_joint_time).nanoseconds / 1e9
        if dt <= 0.0:
            return  # Избегаем деления на ноль или отрицательных dt

        # Обновление позиций суставов
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_positions:
                self.joint_positions[name] = position

        # Обновление предыдущих позиций
        # (Удалена логика вычисления self.linear_velocity из joint_state_callback)

        # Обновление времени для joint_state_callback
        self.last_joint_time = current_time

        if self.verbose:
            self.get_logger().info(f"Joint states updated.")

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

        # Обновление положения на основе linear_velocity из /controller_velocity
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

        # Установка ковариации положения (примерные значения)
        odom.pose.covariance = [
            0.0001, 0.0,    0.0,    0.0, 0.0, 0.0,
            0.0,    0.0001, 0.0,    0.0, 0.0, 0.0,
            0.0,    0.0,    1000000.0, 0.0, 0.0, 0.0,
            0.0,    0.0,    0.0,    1000000.0, 0.0, 0.0,
            0.0,    0.0,    0.0,    0.0,    1000000.0, 0.0,
            0.0,    0.0,    0.0,    0.0,    0.0,    0.0001
        ]

        # Установка скоростей
        odom.twist.twist.linear.x = self.linear_velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.imu_angular_velocity  # Используем угловую скорость из IMU

        # Установка ковариации скоростей (примерные значения)
        odom.twist.covariance = [
            0.0001, 0.0,    0.0,    0.0, 0.0, 0.0,
            0.0,    0.0001, 0.0,    0.0, 0.0, 0.0,
            0.0,    0.0,    1000000.0, 0.0, 0.0, 0.0,
            0.0,    0.0,    0.0,    1000000.0, 0.0, 0.0,
            0.0,    0.0,    0.0,    0.0,    1000000.0, 0.0,
            0.0,    0.0,    0.0,    0.0,    0.0,    0.0001
        ]

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

        # Обновление времени для timer_callback
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
