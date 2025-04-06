# mecanum_nodes/mecanum_nodes/planner_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
from .poly_utils import PolyTraj

def create_cubic_3D(x0, y0, theta0, x1, y1, theta1, T):
    """
    Returns three PolyTraj objects, for x(t), y(t), and theta(t),
    each with zero initial and final velocity.
    
    Parameters:
      x0, y0, theta0 : initial position and heading
      x1, y1, theta1 : final position and heading
      T : segment duration
    """
    # Each initial state is a 2x1 vector: [position, velocity]
    X0 = np.array([[x0], [0.0]])
    X1 = np.array([[x1], [0.0]])
    Y0 = np.array([[y0], [0.0]])
    Y1 = np.array([[y1], [0.0]])
    Theta0 = np.array([[theta0], [0.0]])
    Theta1 = np.array([[theta1], [0.0]])
    
    x_traj = PolyTraj(X0, X1, T)
    y_traj = PolyTraj(Y0, Y1, T)
    theta_traj = PolyTraj(Theta0, Theta1, T)
    return x_traj, y_traj, theta_traj


def plan_waypoints(waypoints, segment_time=2.0):
    """
    Takes a list of (x, y, theta) points and returns a list of segments.
    Each segment is a tuple: (x_traj, y_traj, theta_traj, T_segment)
    where each trajectory is a PolyTraj and T_segment is the duration.
    """
    segments = []
    for i in range(len(waypoints)-1):
        x0, y0, theta0 = waypoints[i]
        x1, y1, theta1 = waypoints[i+1]
        x_traj, y_traj, theta_traj = create_cubic_3D(x0, y0, theta0, x1, y1, theta1, segment_time)
        segments.append((x_traj, y_traj, theta_traj, segment_time))
    return segments


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.publisher_ = self.create_publisher(Twist, 'desired_vel', 10)
        self.timer_period = 0.05  # 20 Hz timer
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        # Example waypoints and a 3-second segment time per leg
        # self.waypoints = [(0, 0, 0), (.8, 0, 0), (.8, 0, 0), (0, 0, 0), (0, 0, 0), (-.8, 0, 0),  (-.8, 0, 0), (0, 0, 0),]
        self.waypoints = [(0, 0, 0), (0, 0, 3), (0, 0, 3), (0, 0, 0),(0, 0, 0), (0, 0, -3), (0, 0, -3), (0, 0, 0)]
        self.segments = plan_waypoints(self.waypoints, segment_time=1.0)
        self.current_segment = 0
        self.t = 0.0

    def timer_callback(self):
        if self.current_segment < len(self.segments):
            x_traj, y_traj, theta_traj, T_segment = self.segments[self.current_segment]
            # Evaluate velocity from the trajectories (order=1 gives derivative)
            vx = x_traj.evaluate(self.t, order=0)
            vy = y_traj.evaluate(self.t, order=0)
            vw = theta_traj.evaluate(self.t, order=0)
            msg = Twist()
            msg.linear.x = float(vx)
            msg.linear.y = float(vy)
            msg.angular.z = float(vw)
            self.publisher_.publish(msg)
            self.get_logger().info(f"Segment {self.current_segment}: t={self.t:.2f}, vx={vx:.2f}, vy={vy:.2f}")
            self.t += self.timer_period
            if self.t > T_segment:
                self.current_segment += 1
                self.t = 0.0
        else:
            # Optionally publish zeros when finished
            msg = Twist()
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

