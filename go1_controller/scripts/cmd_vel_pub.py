#!/usr/bin/env python3
# Modified for ros2 by: abutalipovvv
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class CmdVelToJoy(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_joy')

        # Подписываемся на топик /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        # Публикуем команды на топик /joy_ramped
        self.publisher_ = self.create_publisher(Joy, '/joy_ramped', 10)

        self.get_logger().info("Node started: CmdVelToJoy")

    def cmd_vel_callback(self, msg: Twist):
        # Преобразуем сообщение Twist в сообщение Joy
        joy_msg = Joy()

        # Axes: (0 - yaw, 4 - forward/backward)
        joy_msg.axes = [0.0] * 8
        joy_msg.buttons = [0] * 12

        # Повороты (yaw) на оси 0
        joy_msg.axes[0] = msg.angular.z

        # Движение вперед-назад на оси 4
        joy_msg.axes[4] = msg.linear.x

        # Если скорости по нулям (остановка), устанавливаем другие кнопки
        if msg.linear.x == 0.0 and msg.angular.z == 0.0:
            joy_msg.buttons = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # Остановка
        else:
            joy_msg.buttons = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # Движение

        # Публикуем преобразованное сообщение на топик /joy_ramped
        self.publisher_.publish(joy_msg)

def main(args=None):
    rclpy.init(args=args)

    # Запуск ноды
    node = CmdVelToJoy()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Завершаем работу
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
