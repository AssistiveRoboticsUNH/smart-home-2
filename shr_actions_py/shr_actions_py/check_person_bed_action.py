import os
import time

from shr_msgs.action import CheckPersonInBedRequest
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool


class CheckPersonInBedActionServer(Node):
    def __init__(self):
        super().__init__('check_person_bed_action')
        self.action_server = ActionServer(self, CheckPersonInBedRequest, 'in_bed',
                                          self.callback, cancel_callback=self.cancel_callback)

        self.create_subscription(Bool, 'smartthings_sensors_motion_bed_side',
                                 self.bed_side_motion_callback, 10)

        self.motion_detected = False

    def bed_side_motion_callback(self, msg):
        print(msg.data)
        self.motion_detected = msg.data

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def callback(self, goal_handle):
        result = CheckPersonInBedRequest.Result()
        timeout = goal_handle.request.timeout

        start_time = time.time()

        while time.time() - start_time < timeout:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return CheckPersonInBedRequest.Result()
            if self.motion_detected:
                # print('motion detected true')
                self.get_logger().info('Motion detected next to bed indicating the patient returned to bed afetr leaving at night')
                self.get_logger().info('Goal canceled')
                result.status = "success"
                goal_handle.succeed()
                return result

        result.status = "fail"
        self.get_logger().info('Motion not detected next to bed indicating the patient didnt return to bed afetr leaving at night')
        goal_handle.abort()

        return result


def main(args=None):
    rclpy.init(args=args)

    action_server = CheckPersonInBedActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)

    while True:
        executor.spin_once(timeout_sec=1.0)


if __name__ == '__main__':
    main()
