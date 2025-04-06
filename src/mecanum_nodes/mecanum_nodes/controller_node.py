# mecanum_nodes/mecanum_nodes/controller_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np

from .mecanum_kinematics import MecanumKinematics

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        # Subscribe to the desired velocity topic
        self.subscription = self.create_subscription(
            Twist,
            'desired_vel',
            self.on_twist_msg,
            10
        )
        # Publisher for the computed wheel velocities
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'wheel_vels',
            10
        )
        # Create an instance of your Mecanum kinematics
        self.kinematics = MecanumKinematics()

    def on_twist_msg(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # Create a 3x1 numpy array for body velocities: [vx, vy, wz]
        body_vel = np.array([[vx], [vy], [wz]], dtype=float)

        # body fram trajectory
        # Compute the wheel angular velocities using IK
        wheel_vels = self.kinematics.compute_IK_wheel_velocities(body_vel).flatten()

        # world frame -> body frame trajectory
        # R_world = self.kinematics.compute_body_to_world_R_3D(body_vel).T
        # robot_velocity = R_world @ body_vel

        # vel = np.array([[robot_velocity[0]], [robot_velocity[1]], [robot_velocity[2]]], dtype=float)

        # wheel_vels = self.kinematics.compute_IK_wheel_velocities(vel).flatten()



        # Prepare and publish the result as a Float32MultiArray
        out_msg = Float32MultiArray()
        out_msg.data = wheel_vels.tolist()
        self.publisher.publish(out_msg)

        # self.get_logger().info(
        #     f"Received: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f} --> Wheels: {wheel_vels}"
        # )
        self.get_logger().info(
            f"Received: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f} --> Wheels: "
            f"{wheel_vels[0]:.2f}, {wheel_vels[1]:.2f}, {wheel_vels[2]:.2f}, {wheel_vels[3]:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

