import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import serial
import time
from mecanum_nodes.data_manager import DataManager



class StepperMotorNode(Node):
    def __init__(self):
        super().__init__('stepper_motor_node')
        self.subscription_wheels = self.create_subscription(
            Float32MultiArray,
            'wheel_vels',
            self.motor_command_callback,
            10)
        
        self.subscription_outlet = self.create_subscription(
            Float32,
            'outlet_position',
            self.outlet_callback,
            10)
        
        
        
        self.latest_outlet_value = 0.0

        
        # Open serial port for Teensy communication
        try:
            self.Manager = DataManager(4, '/dev/ttyACM0', 115200, timeout=1)
            # self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            # self.serial_port.dtr = False  # Disable DTR to avoid reset
            # self.serial_port.rts = False  # Disable RTS to avoid reset
            
            time.sleep(2)  # Allow Teensy to initialize
            
            self.get_logger().info('Serial port /dev/ttyACM0 opened successfully with DTR/RTS disabled')

            # === New Functionality ===
            # Setup receiver for 7-length data from Teensy
            self.recv_manager = DataManager(7, '/dev/ttyACM0', 115200, timeout=1)
            self.recv_publisher = self.create_publisher(Float32MultiArray, 'fused_body_vel', 10)
            self.recv_timer = self.create_timer(0.001, self.receive_from_teensy)

            # ==========================

        except serial.SerialException as e:
            self.get_logger().error(f'Unable to open serial port: {e}')
            self.serial_port = None

    def outlet_callback(self, msg):
        self.latest_outlet_value = msg.data
        self.get_logger().info(f'Updated outlet value: {self.latest_outlet_value:.2f}')

    def motor_command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received wheel vels command: {command}')
        merged = list(command) + [self.latest_outlet_value]

        
        self.Manager.pack_and_transmit(merged)
        self.get_logger().info(f'Sent command to Teensy: {merged}')

    # === New Function ===
    def receive_from_teensy(self):
        if self.recv_manager.receive_data():
            recv_data = self.recv_manager.parse_data()
            msg = Float32MultiArray()
            msg.data = recv_data
            self.recv_publisher.publish(msg)
            self.get_logger().info(f'Published telemetry from Teensy: {recv_data}')
    # ====================


def main(args=None):
    rclpy.init(args=args)
    node = StepperMotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
