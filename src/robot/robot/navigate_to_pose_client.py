import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class NavigateToPoseClient(Node):
    def __init__(self) -> None:
        super().__init__('navigate_to_pose_client')

        self.get_logger().info('Starting NavigateToPose client')

        self._action_client: ActionClient = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._current_goal_handle = None

        # Subscribe for external goals
        self._goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self._on_goal_pose, 10)

    def _on_goal_pose(self, pose: PoseStamped) -> None:
        if not self._action_client.server_is_ready():
            self.get_logger().warn('NavigateToPose action server not ready yet')
            return

        # Cancel previous goal if any
        if self._current_goal_handle is not None:
            try:
                self._current_goal_handle.cancel_goal_async()
            except Exception:
                pass

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(
            f"Sending goal to ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}) frame={pose.header.frame_id}"
        )

        send_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback,
        )
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        # Provide light feedback for user visibility
        self.get_logger().debug(
            f"Distance remaining: {fb.distance_remaining:.2f}m | Speed: {fb.speed:.2f}"
        )

    def _result_callback(self, future) -> None:
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f'Navigation finished with status {status} | result: {result}')
        self._current_goal_handle = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigateToPoseClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


