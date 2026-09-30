import math
import numpy as np
from scipy.optimize import minimize
from rbsf_mvsim.algorithms.base_controller import BaseSteeringController

class MPCController(BaseSteeringController):
    """Kinematic Model Predictive Control (MPC) with time-based prediction.

    Allows tuning with adjustable hyperparameters.
    """

    def __init__(self, horizon, dt, wheelbase, max_steer, max_speed, 
                 w_x, w_y, w_yaw, w_delta, w_delta_rate, w_v):
        """Initializes the MPCController.

        Args:
            horizon (int): Prediction horizon.
            dt (float): Time step.
            wheelbase (float): Distance between axles.
            max_steer (float): Maximum steering angle.
            max_speed (float): Maximum speed.
            w_x (float): Cost weight for X tracking error.
            w_y (float): Cost weight for Y tracking error.
            w_yaw (float): Cost weight for yaw tracking error.
            w_delta (float): Cost weight for steering effort.
            w_delta_rate (float): Cost weight for steering smoothness.
            w_v (float): Cost weight for speed tracking error.
        """
        self.horizon = horizon          
        self.dt = dt              
        self.wheelbase = wheelbase        
        self.max_steer = max_steer
        self.max_speed = max_speed

        self.w_x = w_x
        self.w_y = w_y
        self.w_yaw = w_yaw
        self.w_delta = w_delta
        self.w_delta_rate = w_delta_rate
        self.w_v = w_v

    @classmethod
    def from_params(cls, params: dict):
        """Creates an MPC instance from a parameter dictionary.

        Args:
            params (dict): Dictionary with parameters.

        Returns:
            MPCController: Configured instance.
        """
        return cls(
            horizon=params.get('horizon', 10),
            dt=params.get('dt', 0.1),
            wheelbase=params.get('wheelbase', 0.33),
            max_steer=params.get('max_steer', 0.4),
            max_speed=params.get('v_max', 5.0),
            w_x=params.get('w_x', 1.0),
            w_y=params.get('w_y', 5.0),
            w_yaw=params.get('w_yaw', 10.0),
            w_delta=params.get('w_delta', 0.1),
            w_delta_rate=params.get('w_delta_rate', 5.0),
            w_v=params.get('w_v', 1.5)
        )

    def calculate_control(self, car_x, car_y, car_yaw, car_speed, path_points, target_speeds):
        """Calculates the steering angle and target speed using MPC.

        Args:
            car_x (float): X-coordinate of the car.
            car_y (float): Y-coordinate of the car.
            car_yaw (float): Car's orientation in radians.
            car_speed (float): Current speed of the car in m/s.
            path_points (list): List of path points.
            target_speeds (list): Target speeds for each path point.

        Returns:
            tuple:
                - float: Target steering angle in radians.
                - float: Target speed in m/s.
        """
        if not path_points:
            return 0.0, 0.0

        closest_idx = self._find_closest_index(car_x, car_y, path_points)
        
        ref_x, ref_y, ref_yaw, ref_v = self._get_reference_trajectory(
            path_points, target_speeds, closest_idx, self.horizon, self.dt
        )

        def cost_function(controls):
            """Evaluates the cost of a given control sequence."""
            cost = 0.0
            x, y, yaw = car_x, car_y, car_yaw
            
            for i in range(self.horizon):
                steer = controls[2*i]
                v = controls[2*i + 1]
                
                x += v * math.cos(yaw) * self.dt
                y += v * math.sin(yaw) * self.dt
                yaw += (v / self.wheelbase) * math.tan(steer) * self.dt
                
                cost += self.w_x * (x - ref_x[i])**2
                cost += self.w_y * (y - ref_y[i])**2
                
                yaw_diff = yaw - ref_yaw[i]
                yaw_diff = (yaw_diff + math.pi) % (2 * math.pi) - math.pi
                cost += self.w_yaw * (yaw_diff)**2
                
                cost += self.w_delta * (steer**2)
                
                if i > 0:
                    prev_steer = controls[2*(i-1)]
                    cost += self.w_delta_rate * ((steer - prev_steer)**2)
                
                cost += self.w_v * (v - ref_v[i])**2 
                
            return cost

        initial_guess = np.zeros(2 * self.horizon)
        initial_guess[1::2] = max(car_speed, 0.5) 
        
        bounds = []
        for _ in range(self.horizon):
            bounds.append((-self.max_steer, self.max_steer))
            bounds.append((0.01, self.max_speed)) 

        result = minimize(
            cost_function, 
            initial_guess, 
            method='SLSQP', 
            bounds=bounds,
            options={'disp': False, 'maxiter': 50} 
        )

        if not result.success:
            return 0.0, 0.0

        return result.x[0], result.x[1]