import math
import numpy as np
from scipy.optimize import minimize
from scipy.interpolate import interp1d
from rbsf_mvsim.algorithms.base_path_optimizer import BasePathOptimizer

class GeometricPathOptimizer(BasePathOptimizer):
    """Offline optimizer that smooths the trajectory (minimizes curvature) 
    purely through geometry and subsequently calculates a kinematic 
    speed profile with forward and backward passes.
    """

    def __init__(
        self,
        max_deviation,
        v_min,
        v_max,
        lat_accel,
        forward_accel,
        braking_accel,
        closure_threshold,
        curvature_weight,
        alpha_smooth_weight,
        alpha_magnitude_weight,
        distance_weight,
    ):
        """Initializes the GeometricPathOptimizer.

        Args:
            max_deviation (float): Maximum lateral deviation limit in meters.
            v_min (float): Minimum speed limit.
            v_max (float): Maximum speed limit.
            lat_accel (float): Maximum lateral acceleration.
            forward_accel (float): Maximum forward acceleration.
            braking_accel (float): Maximum braking deceleration.
            closure_threshold (float): Threshold to consider the path closed.
            curvature_weight (float): Weight for curvature minimization.
            alpha_smooth_weight (float): Weight for smoothing alpha offsets.
            alpha_magnitude_weight (float): Weight for penalizing large alphas.
            distance_weight (float): Weight for path distance minimization.
        """
        self.max_deviation = max_deviation
        self.closure_threshold = closure_threshold
        self.curvature_weight = curvature_weight
        self.distance_weight = distance_weight
        self.alpha_smooth_weight = alpha_smooth_weight
        self.alpha_magnitude_weight = alpha_magnitude_weight
        
        self.v_min = v_min
        self.v_max = v_max
        self.ay_max = lat_accel
        self.ax_max_forward = forward_accel
        self.ax_max_braking = braking_accel

    @classmethod
    def from_params(cls, params: dict):
        """Instantiates the optimizer reading from a yaml configuration dictionary.

        Args:
            params (dict): Dictionary with parameters.

        Returns:
            GeometricPathOptimizer: Configured instance.
        """
        return cls(
            max_deviation=params.get('max_deviation', 1.0),
            v_min=params.get('v_min', 0.5),
            v_max=params.get('v_max', 15.0),
            lat_accel=params.get('lat_accel', 5.0),
            forward_accel=params.get('forward_accel', 3.0),
            braking_accel=params.get('braking_accel', 6.0),
            closure_threshold=params.get('closure_threshold', 0.25),
            curvature_weight=params.get('curvature_weight', 1.0),
            alpha_smooth_weight=params.get('alpha_smooth_weight', 1.0),
            alpha_magnitude_weight=params.get('alpha_magnitude_weight', 0.01),
            distance_weight=params.get('distance_weight', 0.1),
        )

    def optimize_path(self, path_points):
        """Main flow of the optimizer.

        Args:
            path_points (list): List of points representing the path.

        Returns:
            tuple:
                - list: Optimized path points.
                - list: Target speeds.
        """
        coords = self._extract_coordinates(path_points)
        
        optimized_coords = self._minimize_curvature(coords)
        
        target_speeds = self._generate_speed_profile(optimized_coords)
        
        optimized_path_points = self._rebuild_path(optimized_coords)
        
        return optimized_path_points, target_speeds

    def _minimize_curvature(self, coords):
        """Minimizes the curvature of the given coordinates.

        Args:
            coords (np.ndarray): Path coordinates.

        Returns:
            np.ndarray: Optimized coordinates.
        """
        orig_n = len(coords)
        if orig_n < 3:
            return coords

        is_closed = self._is_closed_path(coords)

        seg_lengths = np.linalg.norm(coords[1:] - coords[:-1], axis=1)
        s_orig = np.zeros(orig_n)
        s_orig[1:] = np.cumsum(seg_lengths)
        total_s = s_orig[-1]

        target_ds = 0.5 
        num_downsampled = max(10, int(total_s / target_ds))
        
        if is_closed:
            s_down = np.linspace(0, total_s, num_downsampled, endpoint=False)
        else:
            s_down = np.linspace(0, total_s, num_downsampled)

        interp_func_x = interp1d(s_orig, coords[:, 0], kind='linear')
        interp_func_y = interp1d(s_orig, coords[:, 1], kind='linear')
        coords_down = np.column_stack((interp_func_x(s_down), interp_func_y(s_down)))

        opt_coords = np.copy(coords_down)
        iterations = 500
        
        w_d = self.distance_weight
        w_c = self.curvature_weight
        total_w = w_d + w_c + 1e-9
        w_d = w_d / total_w
        w_c = w_c / total_w
        
        lr = 0.1

        for _ in range(iterations):
            p_pts = opt_coords
            
            if is_closed:
                p_prev = np.roll(p_pts, 1, axis=0)
                p_next = np.roll(p_pts, -1, axis=0)
                p_prev2 = np.roll(p_pts, 2, axis=0)
                p_next2 = np.roll(p_pts, -2, axis=0)
            else:
                p_prev = np.pad(p_pts[:-1], ((1,0), (0,0)), mode='edge')
                p_next = np.pad(p_pts[1:], ((0,1), (0,0)), mode='edge')
                p_prev2 = np.pad(p_pts[:-2], ((2,0), (0,0)), mode='edge')
                p_next2 = np.pad(p_pts[2:], ((0,2), (0,0)), mode='edge')

            f_dist = (p_prev - 2*p_pts + p_next)
            
            f_curv = -(p_prev2 - 4*p_prev + 6*p_pts - 4*p_next + p_next2)

            opt_coords = p_pts + lr * (w_d * f_dist + w_c * f_curv)

            deltas = opt_coords - coords_down
            dists = np.linalg.norm(deltas, axis=1)
            exceed_mask = dists > self.max_deviation
            
            if np.any(exceed_mask):
                direction = deltas[exceed_mask] / dists[exceed_mask, np.newaxis]
                opt_coords[exceed_mask] = coords_down[exceed_mask] + direction * self.max_deviation

        seg_lengths_opt = np.linalg.norm(opt_coords[1:] - opt_coords[:-1], axis=1)
        s_opt = np.zeros(len(opt_coords))
        s_opt[1:] = np.cumsum(seg_lengths_opt)
        total_s_opt = s_opt[-1]
        
        if is_closed:
            s_even = np.linspace(0, total_s_opt, orig_n, endpoint=False)
            opt_coords_full = np.vstack((opt_coords, opt_coords[0]))
            s_opt_full = np.append(s_opt, total_s_opt)
        else:
            s_even = np.linspace(0, total_s_opt, orig_n)
            opt_coords_full = opt_coords
            s_opt_full = s_opt
            
        interp_x = interp1d(s_opt_full, opt_coords_full[:, 0], kind='linear')
        interp_y = interp1d(s_opt_full, opt_coords_full[:, 1], kind='linear')
        
        final_coords = np.column_stack((interp_x(s_even), interp_y(s_even)))
        
        return final_coords
    
    def _is_closed_path(self, coords):
        """Detects if the path should be treated as a closed loop.

        Args:
            coords (np.ndarray): Path coordinates.

        Returns:
            bool: True if closed, False otherwise.
        """
        if len(coords) < 3:
            return False

        segment_lengths = np.linalg.norm(coords[1:] - coords[:-1], axis=1)
        mean_spacing = float(np.mean(segment_lengths)) if len(segment_lengths) > 0 else 0.0
        dynamic_threshold = max(self.closure_threshold, 0.5 * mean_spacing)
        end_to_start = self._distance(coords[0], coords[-1])
        return end_to_start <= dynamic_threshold

    def _generate_speed_profile(self, path_points_array):
        """Generates a speed profile based on the ALREADY OPTIMIZED geometry.

        Args:
            path_points_array (np.ndarray): Optimized path coordinates.

        Returns:
            list: Target speeds.
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
        """Calculates curvature from three points.

        Args:
            p1, p2, p3 (tuple): (x, y) points.

        Returns:
            float: Curvature.
        """
        area = 0.5 * abs(p1[0]*(p2[1] - p3[1]) + p2[0]*(p3[1] - p1[1]) + p3[0]*(p1[1] - p2[1]))
        a = math.hypot(p1[0]-p2[0], p1[1]-p2[1])
        b = math.hypot(p2[0]-p3[0], p2[1]-p3[1])
        c = math.hypot(p3[0]-p1[0], p3[1]-p1[1])
        if a*b*c == 0: return 0.0
        return (4.0 * area) / (a * b * c)

    def _distance(self, p1, p2):
        """Calculates Euclidean distance between two points."""
        return math.hypot(p1[0]-p2[0], p1[1]-p2[1])
