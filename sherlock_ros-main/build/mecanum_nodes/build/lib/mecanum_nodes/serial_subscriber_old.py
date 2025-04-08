import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time
from mecanum_nodes.data_manager import DataManager



class StepperMotorNode(Node):
    def __init__(self):
        super().__init__('stepper_motor_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'wheel_vels',
            self.motor_command_callback,
            10)
        
        # Open serial port for Teensy communication
        try:
            self.Manager = DataManager(4, '/dev/ttyACM0', 115200, timeout=1)
            # self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            # self.serial_port.dtr = False  # Disable DTR to avoid reset
            # self.serial_port.rts = False  # Disable RTS to avoid reset
            
            time.sleep(2)  # Allow Teensy to initialize
            
            self.get_logger().info('Serial port /dev/ttyACM0 opened successfully with DTR/RTS disabled')
        except serial.SerialException as e:
            self.get_logger().error(f'Unable to open serial port: {e}')
            self.serial_port = None

    def motor_command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Send command to Teensy via serial
        # if self.serial_port and self.serial_port.is_open:
        #     #command_str = f'{command}\n'  # Send command as string with newline
        #     #self.serial_port.write(command_str.encode('utf-8'))
        #     self.Manager.pack_and_transmit(command)
        #     self.get_logger().info(f'Sent command to Teensy: {command}')
        # else:
        #     self.get_logger().error('Serial port not open or unavailable.')
        self.Manager.pack_and_transmit(command)
        self.get_logger().info(f'Sent command to Teensy: {command}')


def main(args=None):
    rclpy.init(args=args)
    node = StepperMotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
