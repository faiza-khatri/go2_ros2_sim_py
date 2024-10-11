#!/usr/bin/env python3
# Author: lnotspotl
# Modified for ros2 by: abutalipovvv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, Imu
from std_msgs.msg import Float64MultiArray  # Импортируем для отправки массива
from RobotController import RobotController
from InverseKinematics import robot_IK

USE_IMU = True
RATE = 45

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__("Robot_Controller")

        # Robot geometry
        body = [0.3762, 0.0935]
        legs = [0.0, 0.08, 0.213, 0.213]

        self.a1_robot = RobotController.Robot(body, legs, USE_IMU)
        self.inverseKinematics = robot_IK.InverseKinematics(body, legs)

        # Публикуем в один топик для всех суставов
        self.joint_command_publisher = self.create_publisher(Float64MultiArray, "/joint_group_controller/commands", 10)

        if USE_IMU:
            self.create_subscription(Imu, "/imu_plugin/out", self.a1_robot.imu_orientation, 10)
        
        self.create_subscription(Joy, "/joy_ramped", self.a1_robot.joystick_command, 10)

        self.timer = self.create_timer(1.0 / RATE, self.control_loop)

    def control_loop(self):
        leg_positions = self.a1_robot.run()
        self.a1_robot.change_controller()

        dx = self.a1_robot.state.body_local_position[0]
        dy = self.a1_robot.state.body_local_position[1]
        dz = self.a1_robot.state.body_local_position[2]
        
        roll = self.a1_robot.state.body_local_orientation[0]
        pitch = self.a1_robot.state.body_local_orientation[1]
        yaw = self.a1_robot.state.body_local_orientation[2]

        try:
            joint_angles = self.inverseKinematics.inverse_kinematics(leg_positions, dx, dy, dz, roll, pitch, yaw)

            # Публикуем массив значений команд
            joint_command_msg = Float64MultiArray()
            joint_command_msg.data = joint_angles  # массив углов суставов

            self.joint_command_publisher.publish(joint_command_msg)

        except Exception as e:
            self.get_logger().error(f"Error in control loop: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
