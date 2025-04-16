import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray
import math
import yaml
from PIL import Image
import numpy as np
import os

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

        #subscirber to /camera_goal
        self._goal_sub = self.create_subscription(
            Float32MultiArray,
            '/camera_goal',
            self.camera_goal_callback,
            10
        )
        self.current_goal = None

        #set up timer to check if goal is set
        self.goal_timeout_check_timer = self.create_timer(2.0, self.check_camera_goal_timeout)
        self.time_since_last_goal = 0.0

        #load map image
        self.map_file = '/home/tariq/sherlock_ros/install/my_bot/share/my_bot/config/map1.yaml'
        with open(self.map_file, 'r') as f:
            map_config = yaml.safe_load(f)
            self.resolution = map_config['resolution']
            self.origin = map_config['origin']  # [x, y, theta]
            image_path = map_config['image']
            if not image_path.startswith('/'):
                # Assume relative path from YAML file directory
                map_dir = os.path.dirname(self.map_file)
                image_path = os.path.join(map_dir, image_path)
            self.map_image = np.array(Image.open(image_path).convert('L'))

    def send_goal_to(self, x, y):
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Planner server not available.")
            return

        start = PoseStamped()
        start.header.frame_id = 'map'
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x = 0.0
        start.pose.position.y = 0.0
        start.pose.orientation.w = 1.0

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal
        goal_msg.start = start

        self._action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    
    def trigger_random_exploration(self):
        height, width = self.map_image.shape
        free_pixels = np.argwhere(self.map_image < 10)  # Free = close to black

        if free_pixels.shape[0] == 0:
            self.get_logger().error("No free pixels found in map!")
            return

        idx = np.random.choice(len(free_pixels))
        px, py = free_pixels[idx]

        # Convert from image pixel (row, col) to world (x, y)
        world_x = self.origin[0] + (py + 0.5) * self.resolution
        world_y = self.origin[1] + ((height - px - 1) + 0.5) * self.resolution

        self.get_logger().info(f"Exploring random goal at: ({world_x:.2f}, {world_y:.2f})")
        self.send_goal_to(world_x, world_y)

    def check_camera_goal_timeout(self):
        # No goal ever received
        if self.current_goal is None:
            self.time_since_last_goal += 2.0
            if self.time_since_last_goal > 10.0:
                self.get_logger().info('No camera goal received — triggering random exploration.')
                self.trigger_random_exploration()
                self.time_since_last_goal = 0.0  # Reset
        else:
            self.time_since_last_goal = 0.0  # Reset if goal was recently valid



    def camera_goal_callback(self, msg):
        if len(msg.data) == 3:
            x, y, z = msg.data
            if not any(math.isnan(val) for val in [x, y, z]):
                self.get_logger().info(f'Received camera goal: ({x:.2f}, {y:.2f})')
                self.current_goal = (x, y)
            else:
                self.get_logger().warn('Received NaN in camera goal — ignoring')
                self.current_goal = None


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

            #if you get a goal, use that otherwise do nothing bcos after 10 seconds, the random exploration will be triggered
            goal_x, goal_y = self.current_goal if self.current_goal else (None, None)
            if goal_x is None:
                return
                
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
