#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robot_msgs.msg import RobotVelocity  # Имя пакета и сообщение

class RobotVelocityHandler(Node):
    def __init__(self):  # Исправлено на __init__
        super().__init__('robot_velocity_handler')  # Исправлено на super().__init__

        # Подписка на топик cmd_vel с сообщениями типа Twist
        self.subscription = self.create_subscription(
            Twist, 
            'cmd_vel',  # Подписываемся на cmd_vel, опубликованный teleop_twist_keyboard
            self.robot_velocity_callback, 
            10)

        # Публикация сообщений типа RobotVelocity
        self.publisher_ = self.create_publisher(RobotVelocity, 'robot_velocity', 10)

        self.get_logger().info("Node started: RobotVelocityHandler")

    def robot_velocity_callback(self, msg: Twist):
        # Логируем полученное сообщение для отладки
        self.get_logger().info(
            f"Received Twist: linear=({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), "
            f"angular=({msg.angular.x}, {msg.angular.y}, {msg.angular.z})"
        )

        # Создаём сообщение типа RobotVelocity
        new_msg = RobotVelocity()

        # Заполняем поля сообщения
        new_msg.robot_id = 1  # Пример: жёстко задаём ID робота

        # Масштабируем и ограничиваем линейные скорости
        new_msg.cmd_vel.linear.x = self.multiply_and_limit(msg.linear.x, 0.035, -1.0, 1.0)
        new_msg.cmd_vel.linear.y = self.multiply_and_limit(msg.linear.y, 0.012, -1.0, 1.0)
        new_msg.cmd_vel.linear.z = msg.linear.z  # Не изменяем, если необходимо

        # Ограничиваем угловую скорость
        new_msg.cmd_vel.angular.x = msg.angular.x  # Не изменяем, если необходимо
        new_msg.cmd_vel.angular.y = msg.angular.y  # Не изменяем, если необходимо
        new_msg.cmd_vel.angular.z = self.limit(msg.angular.z, -0.5, 0.5)

        # Публикуем сообщение
        self.publisher_.publish(new_msg)

        # Логируем публикацию
        self.get_logger().info(
            f"Published RobotVelocity: robot_id={new_msg.robot_id}, "
            f"linear=({new_msg.cmd_vel.linear.x}, {new_msg.cmd_vel.linear.y}, {new_msg.cmd_vel.linear.z}), "
            f"angular=({new_msg.cmd_vel.angular.x}, {new_msg.cmd_vel.angular.y}, {new_msg.cmd_vel.angular.z})"
        )
    def multiply_and_limit(self, value, scale_factor, min_limit, max_limit):
        """
        Умножает значение на scale_factor и ограничивает результат между min_limit и max_limit.

        :param value: Исходное значение.
        :param scale_factor: Коэффициент масштабирования.
        :param min_limit: Минимальное допустимое значение.
        :param max_limit: Максимальное допустимое значение.
        :return: Масштабированное и ограниченное значение.
        """
        scaled_value = value * scale_factor
        limited_value = self.limit_value(scaled_value, min_limit, max_limit)
        self.get_logger().debug(f"Scaled value: {scaled_value}, Limited value: {limited_value}")
        return limited_value

    def scale_and_limit(self, value, scale, min_limit, max_limit):
        """
        Масштабирует значение и ограничивает его в заданном диапазоне.
        """
        scaled_value = value / scale
        return self.limit(scaled_value, min_limit, max_limit)

    def limit_value(self, value, min_limit, max_limit):
        """
        Ограничивает значение заданными пределами.

        :param value: Исходное значение.
        :param min_limit: Минимальное допустимое значение.
        :param max_limit: Максимальное допустимое значение.
        :return: Ограниченное значение.
        """
        if value > max_limit:
            return max_limit
        elif value < min_limit:
            return min_limit
        else:
            return value

    def limit(self, value, min_limit, max_limit):
        """
        Ограничивает значение в заданном диапазоне.
        """
        if value > max_limit:
            return max_limit
        elif value < min_limit:
            return min_limit
        else:
            return value

def main(args=None):
    rclpy.init(args=args)

    # Запуск ноды
    node = RobotVelocityHandler()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Завершаем работу
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':  # Исправлено на __name__ и '__main__'
    main()
