# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray


# class MotorFakerNode(Node):
#     def __init__(self):
#         super().__init__('stepper_motor_node')
#         self.publisher_ = self.create_publisher(Float32MultiArray, 'motor_velocities_and_stepper_position', 10)
#         self.timer = self.create_timer(1.0, self.publish_outlet_position)

#     def publish_outlet_position(self):
#         w1 = 1.0
#         w2 = 2.0
#         w3 = 3.0
#         w4 = 4.0

#         m1 = 5.0

#         arr = [w1, w2, w3, w4, m1]

#         message = Float32MultiArray()
#         message.data = arr
#         self.get_logger().info(f'Publishing: {message.data}')
#         self.publisher_.publish(message)


# def main(args=None):
#     rclpy.init(args=args)
#     node = MotorFakerNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32

class MotorFakerNode(Node):
    def __init__(self):
        super().__init__('motor_faker_node')  # give it a different name
        self.publisher_wheels = self.create_publisher(Float32MultiArray, 'wheel_vels', 10)
        self.publisher_outlet = self.create_publisher(Float32, 'outlet_position', 10)
        self.timer = self.create_timer(1.0, self.publish_fake_data)

    def publish_fake_data(self):
        # Wheel velocities
        wheels_msg = Float32MultiArray()
        wheels_msg.data = [1.0, 2.0, 3.0, 4.0]
        self.publisher_wheels.publish(wheels_msg)
        self.get_logger().info(f'Published wheel velocities: {wheels_msg.data}')

        # Outlet position
        outlet_msg = Float32()
        outlet_msg.data = 5.0
        self.publisher_outlet.publish(outlet_msg)
        self.get_logger().info(f'Published outlet position: {outlet_msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = MotorFakerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
