# my_mecanum_project/planner.py
import numpy as np
from .poly_utils import PolyTraj  # note the relative import from same package

def create_cubic_2D(x0, y0, x1, y1, T):
    """
    Returns two PolyTraj objects, for x(t) and y(t),
    each with zero initial and final velocity.
    """
    X0 = np.array([[x0],[0.0]])  # position, velocity
    X1 = np.array([[x1],[0.0]])
    Y0 = np.array([[y0],[0.0]])
    Y1 = np.array([[y1],[0.0]])

    x_traj = PolyTraj(X0, X1, T)
    y_traj = PolyTraj(Y0, Y1, T)
    return x_traj, y_traj

def plan_waypoints(waypoints, segment_time=2.0):
    """
    Takes list of (x,y) points => returns a list of (x_traj, y_traj, T_segment).
    """
    segments = []
    for i in range(len(waypoints)-1):
        x0, y0 = waypoints[i]
        x1, y1 = waypoints[i+1]
        x_traj, y_traj = create_cubic_2D(x0, y0, x1, y1, segment_time)
        segments.append((x_traj, y_traj, segment_time))
    return segments
