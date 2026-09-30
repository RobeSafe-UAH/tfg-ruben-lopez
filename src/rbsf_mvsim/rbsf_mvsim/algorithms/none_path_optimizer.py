import math
import numpy as np
from scipy.optimize import minimize
from scipy.interpolate import interp1d
from rbsf_mvsim.algorithms.base_path_optimizer import BasePathOptimizer

class NonePathOptimizer(BasePathOptimizer):
    """Path optimizer that performs no optimization.

    Returns the path exactly as it is received. Only calculates the speed profile 
    based on the original geometry without modifying it. Used for testing the original path.
    """

    def __init__(self, v_min, v_max, lat_accel, forward_accel, braking_accel, closure_threshold):
        """Initializes the NonePathOptimizer.

        Args:
            v_min (float): Minimum speed.
            v_max (float): Maximum speed.
            lat_accel (float): Maximum lateral acceleration.
            forward_accel (float): Maximum forward acceleration.
            braking_accel (float): Maximum braking deceleration.
            closure_threshold (float): Threshold to consider the path closed.
        """
        self.v_min = v_min
        self.v_max = v_max
        self.ay_max = lat_accel
        self.ax_max_forward = forward_accel
        self.ax_max_braking = braking_accel
        self.closure_threshold = closure_threshold

    @classmethod
    def from_params(cls, params: dict):
        """Creates an instance from a configuration dictionary.

        Args:
            params (dict): Dictionary with parameters.

        Returns:
            NonePathOptimizer: Configured instance.
        """
        return cls(
            v_min=params.get('v_min', 0.5),
            v_max=params.get('v_max', 3.0),
            lat_accel=params.get('lat_accel', 2.0),
            forward_accel=params.get('forward_accel', 1.0),
            braking_accel=params.get('braking_accel', -1.0),
            closure_threshold=params.get('closure_threshold', 0.25)
        )

    def optimize_path(self, path_points):
        """Processes the path without modifying geometry, only adding a speed profile.

        Args:
            path_points (list): List of path points.

        Returns:
            tuple:
                - list: Original path points.
                - list: Target speeds for each point.
        """
        coords = self._extract_coordinates(path_points)
        optimized_coords = coords
        target_speeds = self._generate_speed_profile(optimized_coords)
        optimized_path_points = self._rebuild_path(optimized_coords)
        return optimized_path_points, target_speeds

    def _is_closed_path(self, coords):
        """Detects if the path should be treated as a closed loop.

        Args:
            coords (np.ndarray): Path coordinates.

        Returns:
            bool: True if the path is closed, False otherwise.
        """
        if len(coords) < 3:
            return False

        segment_lengths = np.linalg.norm(coords[1:] - coords[:-1], axis=1)
        mean_spacing = float(np.mean(segment_lengths)) if len(segment_lengths) > 0 else 0.0
        dynamic_threshold = max(self.closure_threshold, 0.5 * mean_spacing)
        end_to_start = self._distance(coords[0], coords[-1])
        return end_to_start <= dynamic_threshold

    def _generate_speed_profile(self, path_points_array):
        """Generates the speed profile based on the geometry.

        Args:
            path_points_array (np.ndarray): Array of path coordinates.

        Returns:
            list: Generated speed profile.
        """
        n = len(path_points_array)
        if n < 3:
            return [self.v_min] * n
        is_closed = self._is_closed_path(path_points_array)

        max_velocities = [0.0] * n
        
        for i in range(n):
            if is_closed:
                p_prev = path_points_array[i - 1]
                p_next = path_points_array[(i + 1) % n]
            elif i == 0:
                p_prev = path_points_array[0]
                p_next = path_points_array[1]
            elif i == n - 1:
                p_prev = path_points_array[n - 2]
                p_next = path_points_array[n - 1]
            else:
                p_prev = path_points_array[i - 1]
                p_next = path_points_array[i + 1]

            p_curr = path_points_array[i]
            
            curvature = self._calculate_curvature(p_prev, p_curr, p_next)
            
            if curvature < 0.001:
                v_limit = self.v_max 
            else:
                v_limit = math.sqrt(self.ay_max / curvature) 
            
            max_velocities[i] = max(self.v_min, min(self.v_max, v_limit))

        for i in range(1, n):
            dist = self._distance(path_points_array[i], path_points_array[i-1])
            v_prev = max_velocities[i-1]
            
            max_exit_speed = math.sqrt(v_prev**2 + 2 * self.ax_max_forward * dist)
            max_velocities[i] = min(max_velocities[i], max_exit_speed)

        if is_closed:
            dist_end = self._distance(path_points_array[-1], path_points_array[0])
            v_last = max_velocities[-1]
            max_start = math.sqrt(v_last**2 + 2 * self.ax_max_forward * dist_end)
            max_velocities[0] = min(max_velocities[0], max_start)

        final_velocities = list(max_velocities)
        
        for i in range(n - 2, -1, -1):
            dist = self._distance(path_points_array[i], path_points_array[i+1])
            v_next = final_velocities[i+1]
            
            max_entry_speed = math.sqrt(v_next**2 + 2 * self.ax_max_braking * dist)
            final_velocities[i] = min(final_velocities[i], max_entry_speed)

        if is_closed:
            dist_end = self._distance(path_points_array[-1], path_points_array[0])
            v_first = final_velocities[0]
            max_end = math.sqrt(v_first**2 + 2 * self.ax_max_braking * dist_end)
            final_velocities[-1] = min(final_velocities[-1], max_end)

        return final_velocities

    def _calculate_curvature(self, p1, p2, p3):
        """Calculates curvature using circumscribed circle radius."""
        area = 0.5 * abs(p1[0]*(p2[1] - p3[1]) + p2[0]*(p3[1] - p1[1]) + p3[0]*(p1[1] - p2[1]))
        a = math.hypot(p1[0]-p2[0], p1[1]-p2[1])
        b = math.hypot(p2[0]-p3[0], p2[1]-p3[1])
        c = math.hypot(p3[0]-p1[0], p3[1]-p1[1])
        if a*b*c == 0:
            return 0.0
        return (4.0 * area) / (a * b * c)

    def _distance(self, p1, p2):
        """Calculates Euclidean distance between two points."""
        return math.hypot(p1[0]-p2[0], p1[1]-p2[1])
