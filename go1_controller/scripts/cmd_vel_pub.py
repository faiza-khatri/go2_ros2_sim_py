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
        new_msg.cmd_vel = msg  # Копируем сообщение Twist в поле cmd_vel

        # Публикуем сообщение
        self.publisher_.publish(new_msg)

        # Логируем публикацию
        self.get_logger().info(
            f"Published RobotVelocity: robot_id={new_msg.robot_id}, "
            f"linear=({new_msg.cmd_vel.linear.x}, {new_msg.cmd_vel.linear.y}, {new_msg.cmd_vel.linear.z}), "
            f"angular=({new_msg.cmd_vel.angular.x}, {new_msg.cmd_vel.angular.y}, {new_msg.cmd_vel.angular.z})"
        )

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