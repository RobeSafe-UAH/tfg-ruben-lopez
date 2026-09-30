import math
import numpy as np

class SpeedProfiler:
    """Generates a speed profile for a given path based on kinematic limits.
    This class is deprecated and it is not in use now, the speed profile is calculated on the path optimizer.
    """

    def __init__(self, v_min, v_max, lat_accel, long_accel):
        """Initializes the SpeedProfiler.

        Args:
            v_min (float): Minimum speed.
            v_max (float): Maximum speed.
            lat_accel (float): Maximum lateral acceleration (cornering grip).
            long_accel (float): Maximum longitudinal acceleration (braking).
        """
        self.v_min = v_min
        self.v_max = v_max
        self.ay_max = lat_accel
        self.ax_max = long_accel

    def generate_profile(self, path_points):
        """Generates a speed profile that respects braking distances and cornering grip.

        Args:
            path_points (list): List of points (x, y) representing the path.

        Returns:
            list: Target speed profile for the path points.
        """
        if len(path_points) < 3:
            return [self.v_min] * len(path_points)

        n = len(path_points)
        max_velocities = [0.0] * n
        
        for i in range(n):
            p_prev = path_points[i - 1]
            p_curr = path_points[i]
            p_next = path_points[(i + 1) % n]
            
            curvature = self._calculate_curvature(p_prev, p_curr, p_next)
            
            if curvature < 0.01:
                v_limit = self.v_max
            else:
                v_limit = math.sqrt(self.ay_max / curvature)
            
            max_velocities[i] = max(self.v_min, min(self.v_max, v_limit))

        final_velocities = list(max_velocities)
        
        for _ in range(2): 
            for i in range(n - 2, -1, -1):
                dist = self._distance(path_points[i], path_points[i+1])
                v_next = final_velocities[i+1]
                max_entry_speed = math.sqrt(v_next**2 + 2 * self.ax_max * dist)
                final_velocities[i] = min(final_velocities[i], max_entry_speed)

            dist_end = self._distance(path_points[-1], path_points[0])
            v_first = final_velocities[0]
            max_end = math.sqrt(v_first**2 + 2 * self.ax_max * dist_end)
            final_velocities[-1] = min(final_velocities[-1], max_end)

        return final_velocities

    def _calculate_curvature(self, p1, p2, p3):
        """Calculates curvature using the circumscribed circle radius.

        Args:
            p1 (tuple): Point 1 (x, y).
            p2 (tuple): Point 2 (x, y).
            p3 (tuple): Point 3 (x, y).

        Returns:
            float: The curvature.
        """
        area = 0.5 * abs(p1[0]*(p2[1] - p3[1]) + p2[0]*(p3[1] - p1[1]) + p3[0]*(p1[1] - p2[1]))
        a = math.hypot(p1[0]-p2[0], p1[1]-p2[1])
        b = math.hypot(p2[0]-p3[0], p2[1]-p3[1])
        c = math.hypot(p3[0]-p1[0], p3[1]-p1[1])
        if a*b*c == 0:
            return 0.0
        return (4.0 * area) / (a * b * c)

    def _distance(self, p1, p2):
        """Calculates the Euclidean distance between two points.

        Args:
            p1 (tuple): Point 1 (x, y).
            p2 (tuple): Point 2 (x, y).

        Returns:
            float: Distance between the points.
        """
        return math.hypot(p1[0]-p2[0], p1[1]-p2[1])