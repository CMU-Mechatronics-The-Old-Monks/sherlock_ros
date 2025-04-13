import numpy as np
from numpy.typing import NDArray
from enum import Enum
from .poly_utils import Polynomial, PolyTraj
import time
import pygame
import matplotlib.cm as cm
import matplotlib.colors as mcolors

PIXELS_PER_METER = 200
SIM_BACKGROUND_COLOR = (20, 20, 20)
ROBOT_EDGE_COLOR = (0, 150, 255)
ROBOT_HEADING_LINE_COLOR = (255, 100, 100)


class StateIDX():

    '''helper class to index state vector (body frame)'''
    def __init__(self):

        self.x = (0,0)
        self.y = (1,0)
        self.theta = (2,0)
        self.dx = (3,0)
        self.dy = (4,0)
        self.dtheta = (5,0)

        self.position = (slice(0, 2), 0)
        self.pose = (slice(0, 3), 0)

        self.velocity = (slice(3, 6), 0)
        self.linear_velocity = (slice(3, 5),0)

class MecanumKinematics():
    '''
    paper reference: https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf

    NOTE (conventions):
        body +x (forward face of robot - longitudinal)
        body +y (left face of robot - transverse)
        wheel 1 (front left)
        wheel 2 (front right)
        wheel 3 (back left)
        wheel 4 (back right)
    '''

    def __init__(self):

        self.wheel_radius = 0.097/2   #[m]
        self.roller_radius = 0.01   #[m]
        self.roller_angle = np.pi/4 #[rad]
        
        self.width = 0.31           #[m]
        self.length = 0.295           #[m]

        self.state = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])
        self.state_idx = StateIDX()

        self.wheel_vel_vec = np.array([[0.0], [0.0], [0.0], [0.0]])

        lw = (self.width + self.length)
        
        # 4x3
        self.IK_jacobian = (1/self.wheel_radius)*np.array([[1, -1, -lw],
                                                            [1, 1, lw],
                                                            [1, 1, -lw],
                                                            [1, -1, lw],])

        # 3x4
        self.FK_jacobian = (self.wheel_radius/4)*np.array([[1, 1, 1, 1],
                                                            [1, 1, 1, 1],
                                                            [-1/lw, 1/lw, -1/lw, 1/lw]])
        
    def get_robot_position(self, ) -> NDArray[np.float64]:
        '''return the current 2x1 (body frame) planar position vector [[x], [y]]'''
        return np.reshape(self.state[self.state_idx.position], (2,1))
    
    def get_robot_pose(self, ) -> NDArray[np.float64]:
        '''return the current 3x1 (body frame) planar pose vector [[x], [y], [theta]]'''
        return np.reshape(self.state[self.state_idx.pose], (3,1))
        
    def get_robot_linear_velocity(self, ) -> NDArray[np.float64]:
        '''return the current 2x1 (body frame) linear velocity vector [[dx], [dy]]'''
        return np.reshape(self.state[self.state_idx.linear_velocity], (2,1))
    
    def get_robot_angular_velocity(self, ) -> NDArray[np.float64]:
        '''return the current 1, (body is world frame) angular velocity vector [dtheta]'''
        return self.state[self.state_idx.dtheta]
    
    def get_robot_velocity(self, ) -> NDArray[np.float64]:
        '''return the current 3x1 (body frame) planar velocity vector [[dx], [dy], [dtheta]]'''
        return np.reshape(self.state[self.state_idx.velocity], (3,1))
    
    def get_wheel_velocities(self, ) -> NDArray[np.float64]:
        '''return the current 4x1 wheel angular velocity vector [[w1], [w2], [w3], [w4]]'''
        return self.wheel_vel_vec
    
    def get_world_velocity(self, body_velocity: NDArray = None) -> NDArray[np.float64]:
        '''return the current 3x1 (world frame) planar velocity vector [[dx], [dy], [dtheta]]'''
        if body_velocity is not None:
            assert np.shape(body_velocity) == (3, 1)

        else:
            body_velocity = self.get_robot_velocity()

        # unpack
        body_vel = body_velocity[0:2, 0]
        body_vel_longitudinal = body_vel[0]
        body_vel_transverse = body_vel[1]
        body_dtheta = body_velocity[3,0]

        angle = np.arctan(body_vel_transverse/body_vel_longitudinal)
        magnitude = np.linalg.norm(body_vel)

        vx_world = magnitude*np.cos(angle)
        vy_world = magnitude*np.sin(angle)
        if angle >= np.pi:
            vy_world *= -1
            
        return np.array([[vx_world], [vy_world], [body_dtheta]])
    
    def set_robot_position(self, position: NDArray) -> None:
        '''
        set the current 2x1 (body frame) planar position vector
        PARAMS:
            position (NDArray) : 2x1 np.array([[x], [y]])
        '''
        assert np.shape(position) == (2, 1)
        self.state[self.state_idx.position] = position.flatten()

    def set_robot_pose(self, pose: NDArray) -> None:
        '''
        set the current 3x1 (body frame) planar pose vector
        PARAMS:
            pose (NDArray) : 3x1 np.array([[x], [y], [theta]])
        '''
        assert np.shape(pose) == (3, 1)
        self.state[self.state_idx.pose] = pose.flatten()
    
    def set_robot_velocity(self, velocity: NDArray) -> None:
        '''
        set the current 3x1 (body frame) planar velocity vector
        PARAMS:
            velocity (NDArray) : 3x1 np.array([[dx], [dy], [dtheta]])
        '''
        assert np.shape(velocity) == (3, 1)
        self.state[self.state_idx.velocity] = velocity.flatten()

    def set_wheel_velocities(self, angular_velocities: NDArray) -> None:
        '''
        set the current 4x1 wheel velocity vector
        PARAMS:
            angular_velocities (NDArray) : 4x1 np.array([[w1], [w2], [w3], [w4]])
        '''
        assert np.shape(angular_velocities) in [(4,), (4, 1)]
        self.wheel_vel_vec = angular_velocities.flatten()
    
    def compute_FK_robot_velocity(self, angular_velocities: NDArray) -> NDArray[np.float64]:
        '''
        given wheel velocities, compute the resultant nominal body velocity
        PARAMS:
            angular_velocities (NDArray) : 4x1 np.array([[w1], [w2], [w3], [w4]])
        '''
        assert np.shape(angular_velocities) == (4, 1)
        return self.FK_jacobian@angular_velocities
    
    def compute_IK_wheel_velocities(self, body_velocity: NDArray) -> NDArray[np.float64]:
        '''
        given a desired body velocity, compute the required nominal wheel velocities
        PARAMS:
            body_velocity (NDArray) : 3x1 np.array([[dx], [dy], [dtheta]])
        '''
        assert np.shape(body_velocity) == (3, 1)
        return self.IK_jacobian@body_velocity
    
    def compute_body_to_world_R_2D(self, theta: float = None) -> NDArray[np.float64]:
        '''Rotation matrix converting body frame to world frame (2D)'''
        if theta is None:
            theta = self.state[self.state_idx.theta]
        
        c = np.cos(theta)
        s = np.sin(theta)
        
        return np.array([[c, -s], [s, c]])
    
    def compute_body_to_world_R_3D(self, theta: float = None) -> NDArray[np.float64]:
        '''Rotation matrix converting body frame to world frame (3D)'''
        if theta is None:
            theta = self.state[self.state_idx.theta]
        
        c = np.cos(theta)
        s = np.sin(theta)
        
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

class MecanumDynamics():

    def __init__(self):
        raise NotImplementedError
    
        self.wheel_radius = 0.097   #[m]
        self.roller_radius = 0.02   #[m]
        self.roller_angle = np.pi/4 #[rad]
        
        self.width = 0.25           #[m]
        self.length = 0.3           #[m]
    
        self.mass = 8.0             #[kg]

        self.wheel_normal_force = (self.mass*9.81)/4    #[N]

    def get_constraints():
        raise NotImplementedError

    def compute_forward_dynamics():
        raise NotImplementedError
    
    def compute_inverse_dynamics():
        raise NotImplementedError

class MecanumKinematicsSimulator():
    '''toy to simulate kinematics, NO DYNAMICS'''
    def __init__(self, robot: MecanumKinematics):
        self.robot = robot
        self.control_frame = 'body'  # 'world'
        self.fixed_base = False
        self.dt = 0.05  # simulation timestep
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 1.0  # rad/s
        self.pose_history = []

    def draw_robot(self, screen, pose, wheel_velocities):
        """
        Draws the robot as a rotated rectangle with optional wheel indicators.
        pose: np.array([x, y, theta])
        """
        screen_center = np.array(screen.get_size()) / 2

        x, y, theta = pose.flatten()
        px = int(screen_center[0] + x * PIXELS_PER_METER)
        py = int(screen_center[1] - y * PIXELS_PER_METER)  # y is flipped for screen coords

        length = self.robot.length * PIXELS_PER_METER
        width = self.robot.width * PIXELS_PER_METER

        corners = np.array([
            [-length/2, -width/2],  # front-left
            [-length/2,  width/2],  # front-right
            [ length/2, -width/2],  # rear-left
            [ length/2,  width/2],  # rear-right
        ])

        # Rotate corners
        c, s = np.cos(-theta), np.sin(-theta)
        R = np.array([[c, -s], [s, c]])
        heading_point_body = np.array([[length/2], [0]])

        rotated_corners = (R @ corners.T).T + np.array([[px, py]])
        heading_world = (R@heading_point_body).flatten() + np.array([[px, py]])


        hx, hy = heading_world.tolist()[0]

        pygame.draw.polygon(screen, ROBOT_EDGE_COLOR, rotated_corners, 2)

        # Draw heading arrow
        pygame.draw.line(screen, ROBOT_HEADING_LINE_COLOR, (px, py), (hx, hy), 3)

        # Draw arrows for wheel velocities
        arrow_scale = 1  # Scale factor to convert wheel velocity to pixels

        # Arrow directions in body frame: all point along the x-axis
        wheel_dirs_body = np.array([
            [1, 0],  # front-left
            [1, 0],  # front-right
            [1, 0],  # rear-right
            [1, 0]   # rear-left
        ])

        for i in range(4):
            tail = rotated_corners[i]
            # Rotate wheel direction by heading
            direction = R @ wheel_dirs_body[i]  # shape (2,)
            magnitude = wheel_velocities[i]
            head = tail + direction * magnitude * arrow_scale

            pygame.draw.line(
                screen,
                (200, 255, 200),
                tail.astype(int),
                head.astype(int),
                3
            )
            # Optional: draw arrowhead
            pygame.draw.circle(screen, (255, 255, 255), head.astype(int), 3)

    def run_keyboard_input_sim(self):
        pygame.init()
        screen = pygame.display.set_mode((400, 400))
        pygame.display.set_caption("MecanumBot Sim")
        clock = pygame.time.Clock()

        running = True

        while running:
            # Parse events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Key states
            keys = pygame.key.get_pressed()
            vx, vy, wz = 0.0, 0.0, 0.0

            if keys[pygame.K_w]:
                vx += self.linear_speed
            if keys[pygame.K_s]:
                vx -= self.linear_speed
            if keys[pygame.K_d]:
                vy -= self.linear_speed
            if keys[pygame.K_a]:
                vy += self.linear_speed
            if keys[pygame.K_q]:
                wz += self.angular_speed
            if keys[pygame.K_e]:
                wz -= self.angular_speed

            # Build desired velocity in control frame
            desired_vel = np.array([[vx], [vy], [wz]])

            if self.control_frame == 'body':
                robot_velocity = desired_vel
            elif self.control_frame == 'world':
                R_world_to_body = self.robot.compute_body_to_world_R_3D().T
                robot_velocity = R_world_to_body @ desired_vel
            else:
                raise ValueError(f"Unknown control_frame: {self.control_frame}")

            # Compute wheel speeds from IK
            wheel_vels = self.robot.compute_IK_wheel_velocities(robot_velocity)
            self.robot.set_wheel_velocities(wheel_vels)

            # If base is fixed, don't move — just observe wheel speeds
            if not self.fixed_base:
                current_pose = self.robot.get_robot_pose()
                new_pose = current_pose.copy()

                # Update pose using FK
                dx_body = robot_velocity * self.dt
                R_body_to_world = self.robot.compute_body_to_world_R_3D()
                dxy_world = R_body_to_world @ dx_body

                new_pose[0:3, 0] += dxy_world.flatten()
                self.robot.set_robot_pose(new_pose)

            # Example printout for debugging
            print(f"Wheel Speeds: {self.robot.get_wheel_velocities().round(2)} rad/s")

            # Clear screen
            screen.fill((30, 30, 30))

            # Get robot pose and wheel speeds
            pose = self.robot.get_robot_pose()
            wheel_vels = self.robot.get_wheel_velocities()

            # Store pose history for trace
            if not hasattr(self, "pose_history"):
                self.pose_history = []
            self.pose_history.append(pose.copy())

            # Draw trace
            for past_pose in self.pose_history:
                x, y = past_pose[0:2].flatten()
                px = int(screen.get_width() / 2 + x * 100)
                py = int(screen.get_height() / 2 - y * 100)
                pygame.draw.circle(screen, (80, 80, 80), (px, py), 1)

            # Draw robot
            self.draw_robot(screen, pose, wheel_vels)

            # Tick
            pygame.display.flip()
            clock.tick(1.0 / self.dt)

        pygame.quit()

    def compute_drawing_points(self, 
                               poly_traj: list[PolyTraj], 
                               trajectory_frame: str = 'body',
                               control_input: str = 'position') -> list:
        
        T = poly_traj[0].T

        # Precompute expected trajectory points
        N = int(T / self.dt)
        expected_path = []
        x, y, theta = 0.0, 0.0, 0.0

        if control_input == 'velocity':
                diff_order = 0

        elif control_input == 'position':
            diff_order = 1

        for i in range(N):
            ti = i * self.dt
            vx = poly_traj[0].evaluate(ti, order=diff_order)
            vy = poly_traj[1].evaluate(ti, order=diff_order)
            dtheta = poly_traj[2].evaluate(ti, order=diff_order)

            if trajectory_frame == 'body':
                R = self.robot.compute_body_to_world_R_3D(theta)
                dpose = R @ np.array([[vx], [vy], [dtheta]]) * self.dt
                x += dpose[0, 0]
                y += dpose[1, 0]
                theta += dpose[2, 0]

            elif trajectory_frame == 'world':
                x += vx * self.dt
                y += vy * self.dt
                theta += dtheta * self.dt
            else:
                raise ValueError(f"Unknown trajectory_frame: {trajectory_frame}")

            expected_path.append((x, y, theta))

        return expected_path

    def draw_trajectory(self, screen, colormap, norm,  path_points) -> None:

        for x, y, theta in path_points:
            px = int(screen.get_width() / 2 + x * PIXELS_PER_METER)
            py = int(screen.get_height() / 2 - y * PIXELS_PER_METER)
            color = colormap(norm(((theta + np.pi) % (2 * np.pi)) - np.pi))
            color = tuple(int(c * 255) for c in color[:3])
            pygame.draw.circle(screen, color, (px, py), 4)
            pygame.draw.circle(screen, SIM_BACKGROUND_COLOR, (px, py), 3)

    def draw_grid(self, screen, color=(40, 40, 40)):
        width, height = screen.get_size()
        center_x = width // 2
        center_y = height // 2

        # Vertical lines (X axis)
        for x in range(center_x, width, PIXELS_PER_METER):
            pygame.draw.line(screen, color, (x, 0), (x, height))
        for x in range(center_x, 0, -PIXELS_PER_METER):
            pygame.draw.line(screen, color, (x, 0), (x, height))

        # Horizontal lines (Y axis)
        for y in range(center_y, height, PIXELS_PER_METER):
            pygame.draw.line(screen, color, (0, y), (width, y))
        for y in range(center_y, 0, -PIXELS_PER_METER):
            pygame.draw.line(screen, color, (0, y), (width, y))

        # Draw world origin crosshair
        pygame.draw.line(screen, (120, 120, 120), (center_x - 20, center_y), (center_x + 20, center_y), 2)
        pygame.draw.line(screen, (120, 120, 120), (center_x, center_y - 20), (center_x, center_y + 20), 2)
    
    def track_trajectory_sim(
            self, poly_traj: list[PolyTraj], 
            trajectory_frame: str = 'body',
            control_input: str = 'velocity'  # or 'position'
            ):
        
        assert trajectory_frame in ['world', 'body'], "trajectory_frame must be 'world' or 'body'"
        assert control_input in ['velocity', 'position'], "control_input must be 'velocity' or 'position'"
        
        # pygame boiler
        pygame.init()
        pygame.font.init()
        font = pygame.font.SysFont('Arial', 18)
        screen = pygame.display.set_mode((800, 800))
        pygame.display.set_caption("Trajectory Tracking Sim")
        clock = pygame.time.Clock()

        T = poly_traj[0].T
        t = 0.0

        # Precompute expected trajectory points
        N = int(T / self.dt)
        expected_path = self.compute_drawing_points(poly_traj=poly_traj, 
                                                    trajectory_frame=trajectory_frame
                                                    ,control_input=control_input)

        # color map for heading robot angle
        norm = mcolors.Normalize(vmin=-np.pi, vmax=np.pi)
        colormap = cm.get_cmap('hsv')

        # initial conditions
        vx0 = poly_traj[0].evaluate(t)
        vy0 = poly_traj[1].evaluate(t)
        dtheta0 = poly_traj[2].evaluate(t)
        robot_velocity = np.array([[vx0], [vy0], [dtheta0]])

        if control_input == 'velocity':
            diff_order = 0

        elif control_input == 'position':
            diff_order = 1
        
        running = True
        while running and t <= T:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Clear screen
            screen.fill((20, 20, 20))
            self.draw_grid(screen)

            # Draw expected trajectory
            self.draw_trajectory(screen, colormap, norm, expected_path)

            vx = poly_traj[0].evaluate(t, order=diff_order)
            vy = poly_traj[1].evaluate(t, order=diff_order)
            dtheta = poly_traj[2].evaluate(t, order=diff_order)

            desired_vel = np.array([[vx], [vy], [dtheta]])

            if trajectory_frame == 'body':
                robot_velocity = desired_vel

            elif trajectory_frame == 'world':
                R_world_to_body = self.robot.compute_body_to_world_R_3D().T
                robot_velocity = R_world_to_body @ desired_vel

            # Compute wheel speeds and apply
            wheel_vels = self.robot.compute_IK_wheel_velocities(robot_velocity)
            self.robot.set_wheel_velocities(wheel_vels)

            # Integrate pose if not fixed
            if not self.fixed_base:
                current_pose = self.robot.get_robot_pose()
                R = self.robot.compute_body_to_world_R_3D()
                dxy_world = R @ robot_velocity * self.dt
                new_pose = current_pose + dxy_world
                self.robot.set_robot_pose(new_pose)

            # Store history
            self.pose_history.append(self.robot.get_robot_pose().copy())

            # Draw pose history
            for past_pose in self.pose_history:
                x, y, theta = past_pose.flatten()
                px = int((screen.get_width() / 2) + x * PIXELS_PER_METER)
                py = int((screen.get_height() / 2) - y * PIXELS_PER_METER)
                color = colormap(norm(((theta + np.pi) % (2 * np.pi)) - np.pi))
                color = tuple(int(c * 255) for c in color[:3])
                pygame.draw.circle(screen, color, (px, py), 2)

            # Draw robot
            pose = self.robot.get_robot_pose()
            self.draw_robot(screen, pose, wheel_vels)

            time_label = font.render(f"Time: {t:.2f} s", True, (255, 255, 255))
            screen.blit(time_label, (10, 10))
            pygame.display.flip()
            clock.tick(1.0 / self.dt)
            t += self.dt

        pygame.quit()

        

if __name__ == "__main__":

    robot = MecanumKinematics()
    
    
    robot.set_robot_pose(np.array([[0.0], [0.0], [np.pi/2]]))
    sim = MecanumKinematicsSimulator(robot)
    ## keyboard input sim
    sim.run_keyboard_input_sim()


    # # Tracking trajectory sim #
    # T = 1.0
    # traj = [    
    #     PolyTraj(x0=np.array([[0.0], [0.0]]), xf=np.array([[1.0], [0.0]]), T=T), # x or dx
    #     PolyTraj(x0=np.array([[0.0], [0.0]]), xf=np.array([[-1.0], [0.0]]), T=T), # y or dy
    #     PolyTraj(x0=np.array([[0.0], [0.0]]), xf=np.array([[2.0], [0.0]]), T=T)  # theta or dtheta
    # ]
    # sim.track_trajectory_sim(
    #     traj, 
    #     trajectory_frame='body', # trajectory_frame : ['world' | 'body']
    #     control_input='velocity') # control input : ['position' | 'velocity']
