import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np

from .mecanum_kinematics import MecanumKinematics


class FeedbackControllerNode(Node):
    def __init__(self):
        super().__init__('feedback_controller_node')

        self.kinematics = MecanumKinematics()

        self.desired_body_vel = np.zeros((3, 1))  # vx, vy, wz
        self.actual_body_vel = np.zeros((3, 1))   # vx, vy only; wz = 0

        # Subscribers
        self.create_subscription(Twist, 'desired_vel', self.desired_vel_callback, 10)
        self.create_subscription(Float32MultiArray, 'fused_body_vel', self.sensor_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Float32MultiArray, 'wheel_vels_cmd', 10)

        # Loop
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

    def desired_vel_callback(self, msg):
        self.desired_body_vel = np.array([[msg.linear.x], [msg.linear.y], [msg.angular.z]])

    def sensor_callback(self, msg):
        data = np.array(msg.data)
        if data.shape == (2,):
            # Populate only vx and vy, keep wz = 0
            self.actual_body_vel[0, 0] = data[0]  # vx
            self.actual_body_vel[1, 0] = data[1]  # vy
            self.actual_body_vel[2, 0] = 0.0      # no angular vel

    def control_loop(self):
        # Compute error
        vel_error = self.desired_body_vel - self.actual_body_vel

        # Apply correction
        Kp = np.diag([0.6, 0.6, 0.0])  # only correct vx and vy
        corrected_body_vel = self.desired_body_vel + Kp @ vel_error

        # Compute wheel velocities
        corrected_wheel_vels = self.kinematics.compute_IK_wheel_velocities(corrected_body_vel).flatten()

        # Publish
        msg_out = Float32MultiArray()
        msg_out.data = corrected_wheel_vels.tolist()
        self.cmd_pub.publish(msg_out)

        # Log
        self.get_logger().info(
            f"[No WZ] v_des={self.desired_body_vel.flatten().round(2)} | "
            f"v_act={self.actual_body_vel.flatten().round(2)} | "
            f"err={vel_error.flatten().round(2)} | "
            f"wheels={corrected_wheel_vels.round(2)}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FeedbackControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

