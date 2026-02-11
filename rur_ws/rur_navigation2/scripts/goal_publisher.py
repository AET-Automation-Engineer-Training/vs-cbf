#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import time

class SendGoalNode(Node):
    def __init__(self):
        super().__init__('goal_publisher')

        # Action client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publisher /goal_pose (PoseStamped)
        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server found! Sending goal...')

        self.send_goal()

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = 1.0
        goal_msg.pose.pose.position.y = 4.5
        goal_msg.pose.pose.orientation.w = 1.0

        self._action_client.send_goal_async(goal_msg)

        self._goal_pub.publish(goal_msg.pose)
        self.get_logger().info(
            f'Published goal to /goal_pose: ({goal_msg.pose.pose.position.x}, {goal_msg.pose.pose.position.y})'
        )

        time.sleep(1.0)
        self.get_logger().info('Goal sent, shutting down node...')

        # Shutdown node
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    send_goal_node = SendGoalNode()
    rclpy.spin(send_goal_node)


if __name__ == '__main__':
    main()