# my_mecanum_project/main.py

import numpy as np
from .mecanum2 import MecanumKinematics
from .planner import plan_waypoints
from .controller import follow_trajectory

def main():
    # 1) Create the robot
    robot = MecanumKinematics()

    # 2) Define some waypoints
    waypoints = [(0,0), (2,0), (2,2)]
    seg_time = 3.0  # seconds per segment
    segments = plan_waypoints(waypoints, seg_time)

    # 3) Follow the trajectory => compute wheel velocities
    follow_trajectory(robot, segments, dt=0.1)

if __name__ == "__main__":
    main()
