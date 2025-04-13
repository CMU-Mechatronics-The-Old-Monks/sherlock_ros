import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class MotorFakerNode(Node):
    def __init__(self):
        super().__init__('stepper_motor_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'motor_velocities_and_stepper_position', 10)
        self.timer = self.create_timer(1.0, self.publish_outlet_position)

    def publish_outlet_position(self):
        w1 = 1.0
        w2 = 2.0
        w3 = 3.0
        w4 = 4.0

        m1 = 5.0

        arr = [w1, w2, w3, w4, m1]

        message = Float32MultiArray()
        message.data = arr
        self.get_logger().info(f'Publishing: {message.data}')
        self.publisher_.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = MotorFakerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
