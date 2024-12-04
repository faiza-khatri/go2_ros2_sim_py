#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from tf2_ros import TransformBroadcaster
import numpy as np
from math import sin, cos, sqrt
import tf_transformations
import time


class QuadrupedOdometry:
    def __init__(self, body_dimensions, leg_dimensions, imu_data_func, joint_names, node_logger):
        self.body_length = body_dimensions[0]
        self.body_width = body_dimensions[1]

        self.l1, self.l2, self.l3, self.l4 = leg_dimensions

        self.get_imu_data = imu_data_func
        self.joint_names = joint_names

        # Для логирования
        self.logger = node_logger

        # Положение и ориентация робота
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Угол поворота (yaw)

        self.last_support_legs_positions_world = None  # Хранение мировых позиций опорных ног
        self.previous_support_legs = None  # Хранение предыдущих поддерживающих ног

        # Для вычисления скорости
        self.prev_time = None
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Контакты ног (изначально только две ноги на земле для симуляции походки)
        self.support_legs = ['FR', 'RL']  # Пример: Front Right и Rear Left на земле

        # Симуляция смены контакта ног (например, для походки типа "trot")
        self.gait_step = 0

        # Интервал шага в секундах
        self.step_interval = 1.0  # Настройте по необходимости

    def compute_local_positions(self, joint_angles):
        """
        Вычисляет локальные позиции стоп всех ног.
        joint_angles: dict с именами суставов и их углами
        """
        base_points = {
            'FR': np.array([0.5 * self.body_length, -0.5 * self.body_width, 0]),
            'FL': np.array([0.5 * self.body_length, 0.5 * self.body_width, 0]),
            'RR': np.array([-0.5 * self.body_length, -0.5 * self.body_width, 0]),
            'RL': np.array([-0.5 * self.body_length, 0.5 * self.body_width, 0]),
        }

        local_positions = {}

        for leg in ['FR', 'FL', 'RR', 'RL']:
            hip = joint_angles.get(f"{leg}_hip_joint", 0.0)
            thigh = joint_angles.get(f"{leg}_thigh_joint", 0.0)
            calf = joint_angles.get(f"{leg}_calf_joint", 0.0)

            # Прямая кинематика для одной ноги
            x = self.l1 * cos(hip) + self.l2 * cos(hip + thigh) + self.l3 * cos(hip + thigh + calf)
            y = self.l1 * sin(hip) + self.l2 * sin(hip + thigh) + self.l3 * sin(hip + thigh + calf)
            z = 0  # Стопа на земле

            pos = base_points[leg] + np.array([x, y, z])
            local_positions[leg] = pos

        return local_positions

    def update_leg_contacts(self):
        """
        Обновляет состояние контакта ног для симуляции походки.
        В реальной ситуации это должны делать датчики силы или другой источник данных.
        Здесь мы просто чередуем активные ноги для демонстрации.
        """
        # Пример простого чередования шага для походки типа "trot"
        if self.gait_step % 2 == 0:
            self.support_legs = ['FR', 'RL']
        else:
            self.support_legs = ['FL', 'RR']
        self.gait_step += 1

        # Логирование изменения контактов ног
        self.logger.info(f"Gait Step {self.gait_step}: Support legs: {self.support_legs}")

    def update(self, joint_angles, current_time):
        roll, pitch, yaw = self.get_imu_data()

        # Вычисление текущих позиций ног относительно тела
        current_leg_positions = self.compute_local_positions(joint_angles)

        # Вычисление мировых позиций опорных ног
        support_world_positions = []
        for leg in self.support_legs:
            leg_local = current_leg_positions[leg][:2]  # x, y
            # Поворот локальных координат и смещение на (x, y)
            leg_world_x = self.x + cos(self.theta) * leg_local[0] - sin(self.theta) * leg_local[1]
            leg_world_y = self.y + sin(self.theta) * leg_local[0] + cos(self.theta) * leg_local[1]
            support_world_positions.append([leg_world_x, leg_world_y])

        support_world_positions = np.array(support_world_positions)

        if self.support_legs == self.previous_support_legs and self.last_support_legs_positions_world is not None:
            # Вычисляем смещение, чтобы поддерживающие ноги оставались фиксированными
            delta_support = support_world_positions - self.last_support_legs_positions_world

            # Среднее смещение по осям
            delta_x = np.mean(delta_support[:, 0])
            delta_y = np.mean(delta_support[:, 1])

            # Обновление положения тела на противоположную величину смещения опорных ног
            body_shift_x = -delta_x
            body_shift_y = -delta_y

            self.x += body_shift_x
            self.y += body_shift_y

            # Вычисление скорости
            if self.prev_time is not None:
                dt = current_time - self.prev_time
                if dt > 0:
                    self.linear_velocity = sqrt(body_shift_x**2 + body_shift_y**2) / dt
                    self.angular_velocity = (yaw - self.theta) / dt

            # Обновление угла поворота
            self.theta = yaw
            self.prev_time = current_time
        else:
            # Если поддерживающие ноги поменялись, инициализируем новые позиции опорных ног без смещения
            self.prev_time = current_time

        # Сохранение текущих мировых позиций опорных ног для следующего шага
        self.last_support_legs_positions_world = support_world_positions.copy()

        # Обновление предыдущих поддерживающих ног
        self.previous_support_legs = self.support_legs.copy()

        # Логирование обновлённой одометрии
        self.logger.info(f"Updated x: {self.x:.6f}, y: {self.y:.6f}, theta: {self.theta:.6f}")
        self.logger.debug(f"Body Shift X: {body_shift_x if 'body_shift_x' in locals() else 'N/A'}, "
                        f"Body Shift Y: {body_shift_y if 'body_shift_y' in locals() else 'N/A'}, "
                        f"Scale Factor: N/A")


class QuadrupedOdometryNode(Node):
    def __init__(self):
        super().__init__("quadruped_odometry")

        # Параметры
        self.declare_parameter("verbose", False)
        self.declare_parameter("publish_rate", 50)
        self.declare_parameter("imu_topic", "/imu_plugin/out")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_footprint")
        self.declare_parameter("joints", [
            'FR_calf_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'RL_hip_joint', 'FL_hip_joint', 'RL_calf_joint',
            'FR_hip_joint', 'FR_thigh_joint', 'RR_hip_joint',
            'RL_thigh_joint', 'RR_thigh_joint', 'RR_calf_joint',
        ])

        verbose = self.get_parameter("verbose").value
        publish_rate = self.get_parameter("publish_rate").value
        imu_topic = self.get_parameter("imu_topic").value
        joints = self.get_parameter("joints").value

        if verbose:
            self.get_logger().info(f"Publishing rate: {publish_rate}")
            self.get_logger().info(f"IMU topic: {imu_topic}")

        body = [0.3762, 0.0935]  # Примерные размеры тела (длина, ширина)
        legs = [0.0, 0.08, 0.213, 0.213]  # Примерные размеры ног (l1, l2, l3, l4)
        self.odometry = QuadrupedOdometry(body, legs, self.get_imu_data, joints, self.get_logger())

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )
        self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_callback, 10)

        # Подписка на /cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_imu = [0.0, 0.0, 0.0]

        # Скорости из /cmd_vel
        self.cmd_vel_linear = [0.0, 0.0]  # [linear.x, linear.y]
        self.cmd_vel_angular = 0.0  # angular.z

        # Логирование раз в 2 секунды
        self.last_log_time = time.time()
        self.log_interval = 2.0

        # Таймер для публикации одометрии
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_odom)

        # Таймер для обновления контактов ног
        self.step_interval = self.odometry.step_interval
        self.gait_timer = self.create_timer(self.step_interval, self.odometry.update_leg_contacts)

    def get_imu_data(self):
        """
        Возвращает последние значения углов roll, pitch, yaw из IMU.
        """
        return self.last_imu

    def joint_state_callback(self, msg):
        try:
            joint_angles = {name: angle for name, angle in zip(msg.name, msg.position)}
            current_time_msg = self.get_clock().now().to_msg()
            current_time = current_time_msg.sec + current_time_msg.nanosec * 1e-9
            self.odometry.update(joint_angles, current_time)

            if time.time() - self.last_log_time > self.log_interval:
                self.last_log_time = time.time()
                self.get_logger().info(f"JointState: {joint_angles}")
                self.get_logger().info(f"Updated odometry: x={self.odometry.x:.6f}, y={self.odometry.y:.6f}, theta={self.odometry.theta:.6f}")
        except Exception as e:
            self.get_logger().error(f"Error in joint_state_callback: {e}")

    def imu_callback(self, msg):
        try:
            self.last_imu = tf_transformations.euler_from_quaternion([
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ])
            self.get_logger().debug(f"IMU Data (roll, pitch, yaw): {self.last_imu}")
        except Exception as e:
            self.get_logger().error(f"Error in imu_callback: {e}")

    def cmd_vel_callback(self, msg):
        """
        Обновление текущих команд скоростей из /cmd_vel.
        """
        self.cmd_vel_linear = [msg.linear.x, msg.linear.y]
        self.cmd_vel_angular = msg.angular.z

    def publish_odom(self):
        """
        Публикация одометрии, используя /cmd_vel для обновления позиций.
        """
        # Интервал времени
        current_time_msg = self.get_clock().now().to_msg()
        current_time = current_time_msg.sec + current_time_msg.nanosec * 1e-9
        if self.odometry.prev_time is None:
            self.odometry.prev_time = current_time
            return

        dt = current_time - self.odometry.prev_time
        self.odometry.prev_time = current_time

        # Обновление позиций на основе скоростей
        self.odometry.x += self.cmd_vel_linear[0] * cos(self.odometry.theta) * dt - self.cmd_vel_linear[1] * sin(self.odometry.theta) * dt
        self.odometry.y += self.cmd_vel_linear[0] * sin(self.odometry.theta) * dt + self.cmd_vel_linear[1] * cos(self.odometry.theta) * dt
        self.odometry.theta += self.cmd_vel_angular * dt

        # Публикация одометрии
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time_msg
        odom_msg.header.frame_id = self.get_parameter("odom_frame_id").value
        odom_msg.child_frame_id = self.get_parameter("base_frame_id").value
        odom_msg.pose.pose.position.x = self.odometry.x
        odom_msg.pose.pose.position.y = self.odometry.y

        quat = tf_transformations.quaternion_from_euler(0, 0, self.odometry.theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        odom_msg.twist.twist.linear.x = self.cmd_vel_linear[0]
        odom_msg.twist.twist.linear.y = self.cmd_vel_linear[1]
        odom_msg.twist.twist.angular.z = self.cmd_vel_angular

        self.odom_pub.publish(odom_msg)

        # Публикация TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = odom_msg.header.stamp
        tf_msg.header.frame_id = odom_msg.header.frame_id
        tf_msg.child_frame_id = odom_msg.child_frame_id
        tf_msg.transform.translation.x = self.odometry.x
        tf_msg.transform.translation.y = self.odometry.y
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

