import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from shr_msgs.action import PlayVideoRequest  # Replace with your actual action import

import zmq
import time


class SimpleZmqSenderAction(Node):
    def __init__(self, zmq_pub):
        super().__init__('simple_zmq_sender_action')

        self.zmq_pub = zmq_pub
        self._action_server = ActionServer(
            self,
            PlayVideoRequest,
            'play_video',
            execute_callback=self.execute_callback,  # sync version
        )

        self.get_logger().info("✅ SimpleZmqSenderAction is ready.")

    def execute_callback(self, goal_handle):
        video_path = goal_handle.request.file_name
        self.get_logger().info(f"🎯 Received video goal: {video_path}")

        # Optional feedback
        feedback = PlayVideoRequest.Feedback()
        feedback.running = True
        goal_handle.publish_feedback(feedback)

        # Send video path via ZMQ 3 times
        for i in range(3):
            self.zmq_pub.send_string(video_path)
            self.get_logger().info(f"📤 ZMQ sent ({i+1}/3): {video_path}")
            time.sleep(0.3)

        # ⏳ Wait for 55 seconds
        self.get_logger().info("⏳ Waiting 55 seconds before returning success...")
        time.sleep(55)

        goal_handle.succeed()
        result = PlayVideoRequest.Result()
        result.status = "video sent"
        return result


def main(args=None):
    rclpy.init(args=args)

    context = zmq.Context()
    zmq_pub = context.socket(zmq.PUB)
    zmq_pub.bind("tcp://*:5556")  # Android SUB connects here

    node = SimpleZmqSenderAction(zmq_pub)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        zmq_pub.close()
        context.term()


if __name__ == '__main__':
    main()
