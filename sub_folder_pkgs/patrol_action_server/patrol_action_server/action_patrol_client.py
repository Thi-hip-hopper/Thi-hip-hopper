#!/usr/bin/env python3

import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from custom_interfaces2.action import Patrol


class ActionPatrolClient(Node):

    def __init__(self):
        super().__init__('action_patrol_client')
        self._action_client = ActionClient(self, Patrol, 'patrol')

    def send_goal_and_wait(self, radius):
        goal_msg = Patrol.Goal()
        goal_msg.radius = radius

        self.get_logger().info(f'Sending goal with radius: {radius:.2f} meters')

        self._action_client.wait_for_server()

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        result = get_result_future.result().result
        self.get_logger().info(f'Goal completed. Success: {result.success}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Time left: {feedback.time_left:.2f} seconds')


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print("Usage: ros2 run patrol_action_server action_patrol_client <start_radius> <end_radius>")
        return

    try:
        start_radius = float(sys.argv[1])
        end_radius = float(sys.argv[2])
    except ValueError:
        print("Both radius values must be valid floating point numbers.")
        return

    if start_radius > end_radius:
        print("Start radius must be less than or equal to end radius.")
        return

    client = ActionPatrolClient()

    step = 0.1
    if start_radius < end_radius:
        current_radius = start_radius
        while current_radius <= end_radius + 1e-6:
            client.send_goal_and_wait(current_radius)
            current_radius += step
    else:
        current_radius = start_radius
        while current_radius >= end_radius - 1e-6:
            client.send_goal_and_wait(current_radius)
            current_radius -= step

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
