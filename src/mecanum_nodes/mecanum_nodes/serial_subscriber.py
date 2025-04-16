import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import serial
import time
from mecanum_nodes.data_manager import DataManager


class StepperMotorNode(Node):
    def __init__(self):
        super().__init__('stepper_motor_node')

        # Subscriber for wheel velocities (from feedback controller)
        self.subscription_wheels = self.create_subscription(
            Float32MultiArray,
            'wheel_vels_cmd',
            self.motor_command_callback,
            10)

        # Subscriber for outlet position (optional)
        self.subscription_outlet = self.create_subscription(
            Float32,
            'outlet_position',
            self.outlet_callback,
            10)

        self.latest_outlet_value = 0.0

        try:
            self.Manager = DataManager(5, '/dev/ttyACM0', 115200, timeout=1)
            time.sleep(2)  # Let Teensy settle
            self.get_logger().info('Serial port /dev/ttyACM0 opened successfully.')

            # === Telemetry receiver ===
            self.recv_manager = DataManager(7, '/dev/ttyACM0', 115200, timeout=1)  # vx, vy
            self.recv_publisher = self.create_publisher(Float32MultiArray, 'fused_body_vel', 10)
            self.recv_timer = self.create_timer(0.01, self.receive_from_teensy)
            # =========================

        except serial.SerialException as e:
            self.get_logger().error(f'Unable to open serial port: {e}')
            self.Manager = None

    def outlet_callback(self, msg):
        self.latest_outlet_value = msg.data
        self.get_logger().info(f'Updated outlet value: {self.latest_outlet_value:.2f}')

    def motor_command_callback(self, msg):
        command = msg.data
        merged = list(command) + [self.latest_outlet_value]
        self.get_logger().info(f'Sending to Teensy: {merged}')

        if self.Manager:
            self.Manager.pack_and_transmit(merged)
        else:
            self.get_logger().warn('Serial Manager not available.')

    # def receive_from_teensy(self):
    #     if self.recv_manager and self.recv_manager.receive_data():
    #         recv_data = self.recv_manager.parse_data()
    #         msg = Float32MultiArray()
    #         msg.data = recv_data
    #         self.recv_publisher.publish(msg)
    #         self.get_logger().info(f'Published telemetry: {recv_data}')
    
    def receive_from_teensy(self):
        if self.recv_manager.receive_data():
            recv_data = self.recv_manager.parse_data()
            # Extract only vx, vy for now (index 4 and 5)
            vx, vy = recv_data[4], recv_data[5]
            msg = Float32MultiArray()
            msg.data = [vx, vy]
            self.recv_publisher.publish(msg)
            self.get_logger().info(f'Published telemetry: vx={vx:.2f}, vy={vy:.2f}')



def main(args=None):
    rclpy.init(args=args)
    node = StepperMotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
