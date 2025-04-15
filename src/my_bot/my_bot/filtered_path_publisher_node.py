import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray
import math

class FilteredPathPublisher(Node):
    def __init__(self):
        super().__init__('filtered_path_publisher_node')

        self.subscription = self.create_subscription(
            Path,
            '/com_trajectory',
            self.path_callback,
            10
        )
        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            '/filtered_com_trajectory',
            10
        )

        self.get_logger().info("Filtered path publisher is up!")

    def path_callback(self, msg: Path):
        filtered_points = []
        last_x, last_y = None, None
        min_distance = 0.15  # meters

        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y

            if last_x is None:
                filtered_points.append(x)
                filtered_points.append(y)
                last_x, last_y = x, y
                continue

            dist = math.hypot(x - last_x, y - last_y)
            if dist >= min_distance:
                filtered_points.append(x)
                filtered_points.append(y)
                last_x, last_y = x, y

        out_msg = Float32MultiArray()
        out_msg.data = filtered_points
        self.publisher_.publish(out_msg)
        self.get_logger().info(f"Published {len(filtered_points)//2} filtered waypoints.")
        self.get_logger().info(f"Filtered points: {filtered_points}")

def main(args=None):
    rclpy.init(args=args)
    node = FilteredPathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
