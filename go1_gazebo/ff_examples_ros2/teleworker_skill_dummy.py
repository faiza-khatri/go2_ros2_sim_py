#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from teleworker_msgs.srv import StartSkill
import time
import sys

class AddTwoIntsService(Node):

    def __init__(self, service_name):
        super().__init__('teleworker_skill')
        self.srv = self.create_service(StartSkill, service_name, self.service_callback)
        self.get_logger().info('Service server has been started.')

    def service_callback(self, request, response):
        response.success = True
        response.data = "The response message"
        time.sleep(5.0)
        self.get_logger().info(f'Incoming request\ntask_is: {request.task_list_id} name: {request.task_list_name}')
        self.get_logger().info(f'Sending response: {response.data}')
        return response

def main():
    # Read the command-line arguments
    args = sys.argv

    # Ensure at least the script name and one argument are present
    if len(args) < 2:
        print("Usage: python simple_example.py <filename> [-v] [-n <number>]")
        sys.exit(1)
    rclpy.init(args=args)
    node = AddTwoIntsService(args[1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Service server shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
