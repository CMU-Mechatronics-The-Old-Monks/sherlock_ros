# my_mecanum_project/controller.py

import time
import numpy as np
from .mecanum2 import MecanumKinematics

def follow_trajectory(robot, segments, dt=0.05):
    """
    segments: list of (x_traj, y_traj, T_segment)
    dt: time step
    """
    for seg_index, (x_traj, y_traj, T_segment) in enumerate(segments):
        t = 0.0
        print(f"=== Starting segment {seg_index} ===")
        while t <= T_segment:
            vx = x_traj.evaluate(t, order=1)  # dx/dt
            vy = y_traj.evaluate(t, order=1)  # dy/dt
            wz = 0.0  # example: no rotation

            desired_vel = np.array([[vx],[vy],[wz]])
            wheel_vels = robot.compute_IK_wheel_velocities(desired_vel)

            # Do something with wheel_vels (print, serial, etc.)
            print(f"time={t:.2f} => vx={vx:.2f} vy={vy:.2f} => wheels={wheel_vels.round(2)}")

            time.sleep(dt)
            t += dt
        print(f"=== Finished segment {seg_index} ===")
