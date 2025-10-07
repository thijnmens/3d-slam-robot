import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class SimpleNavigateToPoseClient(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_client')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('NavigateToPose client node started')

    def send_goal(self, pose_stamped: PoseStamped):
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available')
            return None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        send_goal_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return None

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        self.get_logger().info(f'Result received: {result.status}')
        return result


def main(argv=None):
    rclpy.init(args=argv)
    node = SimpleNavigateToPoseClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
