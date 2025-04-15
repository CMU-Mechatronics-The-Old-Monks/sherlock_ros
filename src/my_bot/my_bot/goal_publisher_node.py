import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path

class GoalPublisherNode(Node):
    def __init__(self):
        super().__init__('goal_publisher_node')

        # Action client for path planner
        self._action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        # Publisher to /com_trajectory
        self._path_pub = self.create_publisher(Path, 'com_trajectory', 10)

        # Timer to delay goal sending
        self.timer = self.create_timer(5.0, self.send_goal)  # Wait 5 seconds after startup
        self.goal_sent = False

    def send_goal(self):
        if not self.goal_sent and self._action_client.wait_for_server(timeout_sec=2.0):
            self.goal_sent = True
            self.get_logger().info('Sending goal to planner...')

            # Define start and goal poses
            start = PoseStamped()
            start.header.frame_id = 'map'
            start.header.stamp = self.get_clock().now().to_msg()
            start.pose.position.x = 0.0
            start.pose.position.y = 0.0
            start.pose.orientation.w = 1.0

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = 1.0
            goal.pose.position.y = 1.0
            goal.pose.orientation.w = 1.0

            goal_msg = ComputePathToPose.Goal()
            goal_msg.goal = goal
            goal_msg.start = start

            self._action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected!')
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Received plan. Publishing to /com_trajectory')
        self._path_pub.publish(result.path)

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
