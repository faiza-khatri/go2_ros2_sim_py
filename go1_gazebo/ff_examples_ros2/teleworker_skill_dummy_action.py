#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from teleworker_msgs.action import StartSkill
from teleworker_msgs.action._start_skill import StartSkill_Feedback, StartSkill_Result
import time
import sys

class StartSkillActionServer(Node):

    def __init__(self):
        super().__init__('start_skill_action_server')
        self.declare_parameter("action_name", "/test_skill")
        self.declare_parameter("return_success", True)
        self.action_name = self.get_parameter("action_name").get_parameter_value().string_value
        self.return_success = self.get_parameter("action_name").get_parameter_value().bool_value
        self._action_server = ActionServer(
            self,
            StartSkill,
            self.action_name,
            self.execute_callback)
        self.get_logger().info(f'Action server started: {self.action_name}')

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing goal for robot_id: {goal_handle.request.robot_id}...')
        
        feedback_msg = StartSkill_Feedback()
        feedback_msg.status = 'Processing'

        for i in range(5):
            feedback_msg.status = f'Processing step {i+1}/5'
            self.get_logger().info(f'Feedback: {feedback_msg.status}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        if self.return_success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        result = StartSkill_Result()
        result.error_code = StartSkill_Result.NONE
        result.data = 'Task completed successfully.'
        result.success = self.return_success
        return result


def main():
        # Read the command-line arguments
    args = sys.argv

    # # Ensure at least the script name and one argument are present
    # if len(args) < 2:
    #     print("Usage: python simple_example.py <filename> [-v] [-n <number>]")
    #     sys.exit(1)
        
    rclpy.init(args=args)
    node = StartSkillActionServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
