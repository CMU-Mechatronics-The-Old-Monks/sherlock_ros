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
        self.latest_encoders = np.zeros((4, 1))   # w1–w4
        self.latest_imu_v = np.zeros((2, 1))      # vx_imu, vy_imu

        # Adaptive fusion weight
        self.alpha = 0.7         # Start trusting encoders more
        self.alpha_step = 0.02   # Tuning parameter
        self.min_alpha = 0.3     # Minimum trust in encoders
        self.error_threshold = 0.1  # m/s

        self.create_subscription(Twist, 'desired_vel', self.desired_vel_callback, 10)
        self.create_subscription(Float32MultiArray, 'fused_body_vel', self.sensor_callback, 10)

        self.cmd_pub = self.create_publisher(Float32MultiArray, 'wheel_vels_cmd', 10)
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

    def desired_vel_callback(self, msg):
        self.desired_body_vel = np.array([[msg.linear.x], [msg.linear.y], [msg.angular.z]])

    def sensor_callback(self, msg):
        data = np.array(msg.data)
        if data.shape == (7,):
            self.latest_encoders = data[0:4].reshape((4, 1))
            self.latest_imu_v = data[4:6].reshape((2, 1))
            # data[6] = yaw (not used)

    def control_loop(self):
        # FK from encoders
        v_fk = self.kinematics.compute_FK_robot_velocity(self.latest_encoders)
        vx_fk, vy_fk = v_fk[0, 0], v_fk[1, 0]

        # Fused velocity
        vx = self.alpha * vx_fk + (1 - self.alpha) * self.latest_imu_v[0, 0]
        vy = self.alpha * vy_fk + (1 - self.alpha) * self.latest_imu_v[1, 0]
        actual_body_vel = np.array([[vx], [vy], [0.0]])

        # Compute error
        vel_error = self.desired_body_vel - actual_body_vel
        err_norm = np.linalg.norm(vel_error[0:2])  # ignore wz

        # Adjust alpha dynamically
        if err_norm > self.error_threshold:
            self.alpha = min(self.alpha + self.alpha_step, 1.0)
        else:
            self.alpha = max(self.alpha - self.alpha_step, self.min_alpha)

        # Proportional correction
        Kp = np.diag([0.6, 0.6, 0.0])
        corrected_body_vel = self.desired_body_vel + Kp @ vel_error

        # IK to compute final wheel commands
        corrected_wheel_vels = self.kinematics.compute_IK_wheel_velocities(corrected_body_vel).flatten()

        msg_out = Float32MultiArray()
        msg_out.data = corrected_wheel_vels.tolist()
        self.cmd_pub.publish(msg_out)

        # Logging
        self.get_logger().info(
            f"[Adaptive α={self.alpha:.2f}] "
            f"v_des={self.desired_body_vel[0,0]:.2f},{self.desired_body_vel[1,0]:.2f} | "
            f"v_fk={vx_fk:.2f},{vy_fk:.2f} | "
            f"v_imu={self.latest_imu_v[0,0]:.2f},{self.latest_imu_v[1,0]:.2f} | "
            f"v_fused={vx:.2f},{vy:.2f} | "
            f"err={err_norm:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FeedbackControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


