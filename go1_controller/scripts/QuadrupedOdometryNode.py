#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from std_msgs.msg import Float64, Int64
from rosgraph_msgs.msg import Clock  # Для обработки /clock
from builtin_interfaces.msg import Time  # Импорт Time
import tf_transformations
import tf2_ros
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class DogOdometry(Node):
    def __init__(self):
        super().__init__('dog_odometry')
        try:
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

            self.declare_parameter('base_frame_id', 'base')
            self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
            if self.verbose:
                self.get_logger().info(f"Base frame ID: {self.base_frame_id}")

            self.declare_parameter('odom_frame_id', 'odom')
            self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
            if self.verbose:
                self.get_logger().info(f"Odom frame ID: {self.odom_frame_id}")

            # Добавленные параметры для обработки /clock
            self.declare_parameter('is_gazebo', True)
            self.is_gazebo = self.get_parameter('is_gazebo').get_parameter_value().bool_value
            if self.verbose:
                self.get_logger().info(f"Is Gazebo: {self.is_gazebo}")

            self.declare_parameter('clock_topic', '/clock')
            clock_topic = self.get_parameter('clock_topic').get_parameter_value().string_value
            if self.verbose:
                self.get_logger().info(f"Clock Topic: {clock_topic}")

            # Инициализация переменных одометрии
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0  # Будет обновляться из IMU

            # Инициализация времени
            self.last_position_time = self.get_clock().now()

            # Инициализация переменных для /clock и encoder
            self.gazebo_clock = Time()  # Инициализируем как Time
            self.encoder_pos = 0

            # Определение QoS профилей
            qos_reliable = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                depth=10,
                history=HistoryPolicy.KEEP_LAST
            )
            qos_best_effort = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=10,
                history=HistoryPolicy.KEEP_LAST
            )

            # Публикация одометрии с QoS профилем RELIABLE
            self.odom_pub = self.create_publisher(Odometry, 'odom', qos_reliable)

            # Подписка на IMU с QoS профилем RELIABLE
            if self.has_imu_heading:
                self.imu_sub = self.create_subscription(
                    Imu,
                    'imu_plugin/out',
                    self.imu_callback,
                    qos_reliable
                )

            # Подписка на /controller_velocity с QoS профилем RELIABLE
            self.controller_velocity_sub = self.create_subscription(
                Twist,
                'controller_velocity',
                self.controller_velocity_callback,
                qos_reliable
            )

            # Транслятор TF
            if self.enable_odom_tf:
                self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

            # Подписка на /clock или encoder_value в зависимости от параметра is_gazebo
            if self.is_gazebo:
                # Определение QoS профиля для /clock как BEST_EFFORT
                clock_qos = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                    depth=10,
                    history=HistoryPolicy.KEEP_LAST
                )
                self.clock_sub = self.create_subscription(
                    Clock,
                    clock_topic,
                    self.clock_callback,
                    clock_qos  # Используем настроенный QoS профиль
                )
                if self.verbose:
                    self.get_logger().info("Subscribed to /clock topic with BEST_EFFORT QoS.")
            else:
                # Подписка на encoder_value с QoS профилем RELIABLE
                encoder_qos = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                    depth=10,
                    history=HistoryPolicy.KEEP_LAST
                )
                self.encoder_sub = self.create_subscription(
                    Int64,
                    'encoder_value',
                    self.encoder_callback,
                    encoder_qos  # Используем настроенный QoS профиль
                )
                if self.verbose:
                    self.get_logger().info("Subscribed to encoder_value topic with BEST_EFFORT QoS.")

            # Таймер для публикации одометрии
            timer_period = 1.0 / publish_rate
            self.timer = self.create_timer(timer_period, self.timer_callback)

            self.get_logger().info("Dog Odometry Node has been started.")

            # Ограничения на скорости
            self.MAX_LINEAR_VELOCITY = 1.0  # Максимальная линейная скорость в м/с
            self.MAX_ANGULAR_VELOCITY = 2.0  # Максимальная угловая скорость в рад/с

            # Инициализация переменных для хранения угловой скорости из IMU
            self.imu_angular_velocity = 0.0

        except Exception as e:
            self.get_logger().error(f"Exception during node initialization: {e}")
            raise e

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

    # Колбэк для обработки сообщений /clock
    def clock_callback(self, msg):
        self.gazebo_clock = msg.clock
        if self.verbose:
            self.get_logger().info(f"Received Gazebo Clock: {self.gazebo_clock.sec}.{self.gazebo_clock.nanosec}")

    # Колбэк для обработки encoder_value
    def encoder_callback(self, msg):
        self.encoder_pos = msg.data
        if self.verbose:
            self.get_logger().info(f"Received Encoder Position: {self.encoder_pos}")

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_position_time).nanoseconds / 1e9
        if dt <= 0.0:
            return  # Избегаем деления на ноль или отрицательных dt

        # Ограничение скорости
        self.linear_velocity = max(min(self.linear_velocity, self.MAX_LINEAR_VELOCITY), -self.MAX_LINEAR_VELOCITY)
        self.imu_angular_velocity = max(min(self.imu_angular_velocity, self.MAX_ANGULAR_VELOCITY), -self.MAX_ANGULAR_VELOCITY)

        # Обновление положения на основе linear_velocity
        delta_x = self.linear_velocity * math.cos(self.theta) * dt
        delta_y = self.linear_velocity * math.sin(self.theta) * dt
        self.x += delta_x
        self.y += delta_y

        # Создание сообщения одометрии
        odom = Odometry()
        if self.is_gazebo:
            # Используем время из /clock
            odom.header.stamp = self.gazebo_clock
        else:
            # Используем текущее время системы
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
            if self.is_gazebo:
                t.header.stamp = self.gazebo_clock
            else:
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
    try:
        node = DogOdometry()
    except Exception as e:
        print(f"Exception during node initialization: {e}")
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
