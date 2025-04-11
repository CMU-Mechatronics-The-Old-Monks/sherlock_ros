import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class CameraFakerNode(Node):
    def __init__(self):
        super().__init__('stepper_motor_node')
        self.publisher_ = self.create_publisher(Float32, 'outlet_position', 10)
        self.timer = self.create_timer(1.0, self.publish_outlet_position)

    def publish_outlet_position(self):
        message = Float32()
        message.data = 50.0
        self.get_logger().info(f'Publishing: {message.data}')
        self.publisher_.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = CameraFakerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
