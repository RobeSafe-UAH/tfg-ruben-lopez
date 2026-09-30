import math
import numpy as np
from scipy.optimize import minimize
from rbsf_mvsim.algorithms.base_path_optimizer import BasePathOptimizer

class KinematicPathOptimizer(BasePathOptimizer):
    """Kinematic Path Optimizer.

    Finds the optimal sequence of steering and speed that maximizes forward progress
    without leaving the track margins, based on the vehicle's kinematic physics.
    """

    def __init__(self, dt, num_steps, wheelbase, max_steer, max_speed, max_dev):
        """Initializes the KinematicPathOptimizer.

        Args:
            dt (float): Time step.
            num_steps (int): Total horizon to optimize.
            wheelbase (float): Distance between axles.
            max_steer (float): Maximum steering angle.
            max_speed (float): Maximum speed.
            max_dev (float): Maximum allowed deviation from the centerline.
        """
        self.dt = dt
        self.num_steps = num_steps
        self.wheelbase = wheelbase
        self.max_steer = max_steer
        self.max_speed = max_speed
        self.max_dev = max_dev

        self.w_speed = 1.0
        self.w_track = 50.0
        self.w_smooth = 0.5

    @classmethod
    def from_params(cls, params: dict):
        """Creates an instance from a configuration dictionary.

        Args:
            params (dict): Dictionary with parameters.

        Returns:
            KinematicPathOptimizer: Configured instance.
        """
        return cls(
            dt=params.get('dt', 0.1),
            num_steps=params.get('num_steps', 100),
            wheelbase=params.get('wheelbase', 0.33),
            max_steer=params.get('max_steer', 0.4),
            max_speed=params.get('max_speed', 5.0),
            max_dev=params.get('max_deviation', 1.0)
        )

    def optimize_path(self, center_line_points):
        """Optimizes the path based on kinematic limits.

        Args:
            center_line_points (list): List of points representing the centerline.

        Returns:
            tuple:
                - list: Optimized path points.
                - list: Target speeds.
        """
        ref_coords = self._extract_coordinates(center_line_points)
        start_x, start_y = ref_coords[0][0], ref_coords[0][1]
        
        dx = ref_coords[1][0] - ref_coords[0][0]
        dy = ref_coords[1][1] - ref_coords[0][1]
        start_yaw = math.atan2(dy, dx)

        def simulate_kinematics(controls):
            """Simulates the car's trajectory given a sequence of controls."""
            x, y, yaw = start_x, start_y, start_yaw
            path_x, path_y = [x], [y]
            
            for i in range(self.num_steps):
                steer = controls[2*i]
                v = controls[2*i + 1]
                
                x += v * math.cos(yaw) * self.dt
                y += v * math.sin(yaw) * self.dt
                yaw += (v / self.wheelbase) * math.tan(steer) * self.dt
                
                path_x.append(x)
                path_y.append(y)
                
            return np.array(path_x), np.array(path_y)

        def objective(controls):
            """Objective function for the optimizer."""
            sim_x, sim_y = simulate_kinematics(controls)
            
            velocities = controls[1::2]
            cost_speed = -self.w_speed * np.sum(velocities)
            
            steers = controls[0::2]
            steer_diffs = np.diff(steers)
            cost_smooth = self.w_smooth * np.sum(steer_diffs**2)
            
            cost_track = 0.0
            for i in range(len(sim_x)):
                dists = np.sqrt((ref_coords[:,0] - sim_x[i])**2 + (ref_coords[:,1] - sim_y[i])**2)
                min_dist = np.min(dists)
                
                if min_dist > self.max_dev:
                    cost_track += self.w_track * (min_dist - self.max_dev)**2
                    
            return cost_speed + cost_smooth + cost_track

        bounds = []
        for _ in range(self.num_steps):
            bounds.append((-self.max_steer, self.max_steer))
            bounds.append((0.01, self.max_speed))

        initial_guess = np.zeros(2 * self.num_steps)
        initial_guess[1::2] = 1.0

        print(f"Starting Kinematic LTO for {self.num_steps} steps...")
        result = minimize(
            objective, 
            initial_guess, 
            method='L-BFGS-B',
            bounds=bounds,
            options={'maxiter': 200, 'disp': True}
        )
        
        opt_controls = result.x
        opt_x, opt_y = simulate_kinematics(opt_controls)
        target_speeds = opt_controls[1::2].tolist()
        
        target_speeds.append(target_speeds[-1]) 
        
        optimized_coords = np.column_stack((opt_x, opt_y))
        optimized_path_points = self._rebuild_path(optimized_coords)

        return optimized_path_points, target_speeds