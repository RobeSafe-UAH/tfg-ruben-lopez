import math
import numpy as np
from abc import ABC, abstractmethod


class BaseSteeringController(ABC):
    """Abstract base class for steering control algorithms.

    Defines the common interface and provides shared utilities for steering controllers.
    """

    @classmethod
    @abstractmethod
    def from_params(cls, params: dict):
        """Creates a controller instance from a parameter dictionary.

        Args:
            params (dict): Dictionary containing the controller parameters, typically from a YAML configuration.

        Returns:
            BaseSteeringController: A configured instance of the controller.
        """
        ...

    @abstractmethod
    def calculate_control(self, car_x, car_y, car_yaw, car_speed, path_points):
        """Calculates the necessary steering angle to follow the path.

        Args:
            car_x (float): X-coordinate of the car's position.
            car_y (float): Y-coordinate of the car's position.
            car_yaw (float): Car's orientation in radians.
            car_speed (float): Current speed of the car in m/s.
            path_points (list): List of path points (e.g., PoseStamped objects).

        Returns:
            tuple:
                - float: Target steering angle in radians, bounded to [-MAX_STEER, MAX_STEER].
                - int: Index of the closest point on the path to the car.
        """
        ...

    def _find_closest_index(self, car_x, car_y, path_points):
        """Finds the index of the closest path point to the car.

        Args:
            car_x (float): X-coordinate of the car.
            car_y (float): Y-coordinate of the car.
            path_points (list): List of path points.

        Returns:
            int: The index of the closest point.
        """
        min_dist = math.inf
        min_index = 0
        for i, pose in enumerate(path_points):
            px = pose.pose.position.x
            py = pose.pose.position.y
            dist = math.sqrt((px - car_x) ** 2 + (py - car_y) ** 2)
            if dist < min_dist:
                min_dist = dist
                min_index = i
        return min_index

    def _find_target_point(self, car_x, car_y, path_points, from_index, lookahead):
        """Finds the first point from `from_index` that is at least `lookahead` distance away.

        Assumes the path is a closed loop. Searches forward up to one full lap.

        Args:
            car_x (float): X-coordinate of the car.
            car_y (float): Y-coordinate of the car.
            path_points (list): List of path points.
            from_index (int): The index to start searching from.
            lookahead (float): The minimum distance for the target point.

        Returns:
            tuple: (x, y) coordinates of the target point.
        """
        path_len = len(path_points)
        for step in range(1, path_len):
            i = (from_index + step) % path_len
            px = path_points[i].pose.position.x
            py = path_points[i].pose.position.y
            dist = math.sqrt((px - car_x) ** 2 + (py - car_y) ** 2)
            if dist >= lookahead:
                return (px, py)
                
        # Fallback: if no point is far enough, return the point just behind the car
        fallback_idx = (from_index - 1) % path_len
        return (path_points[fallback_idx].pose.position.x, path_points[fallback_idx].pose.position.y)

    def _get_reference_trajectory(self, path_points, target_speeds, start_idx, num_points, dt):
        """Extracts a time-based reference trajectory.

        Interpolates to obtain `num_points` spaced exactly by `dt` seconds.

        Args:
            path_points (list): List of path points.
            target_speeds (list): Target speeds for each path point.
            start_idx (int): The starting index on the path.
            num_points (int): The number of points to extract.
            dt (float): Time interval between points in seconds.

        Returns:
            tuple:
                - list: X-coordinates of the reference trajectory.
                - list: Y-coordinates of the reference trajectory.
                - list: Yaw angles of the reference trajectory.
                - list: Target speeds for the reference trajectory.
        """
        path_len = len(path_points)
        if path_len == 0 or num_points <= 0:
            return [], [], [], []

        # Ensure start_idx is within bounds
        start_idx = start_idx % path_len

        raw_x, raw_y, raw_v, raw_times = [], [], [], []
        total_time = 0.0
        
        prev_x = path_points[start_idx].pose.position.x
        prev_y = path_points[start_idx].pose.position.y
        
        # Assume closed path: we can search ahead up to 2 laps to fill the prediction horizon
        max_search_steps = path_len * 2
        
        for step in range(max_search_steps):
            curr_idx = (start_idx + step) % path_len
            px = path_points[curr_idx].pose.position.x
            py = path_points[curr_idx].pose.position.y
            
            # Get target speed using wrapped index
            if curr_idx < len(target_speeds):
                v = target_speeds[curr_idx]
            elif len(target_speeds) > 0:
                v = target_speeds[-1]
            else:
                v = 0.0
            
            if step == 0:
                raw_times.append(0.0)
            else:
                dist = math.hypot(px - prev_x, py - prev_y)
                if dist < 1e-6:
                    continue
                    
                # Limit minimum speed to avoid division by zero
                safe_v = max(v, 0.1) 
                time_to_travel = dist / safe_v
                total_time += time_to_travel
                raw_times.append(total_time)
                
            raw_x.append(px)
            raw_y.append(py)
            raw_v.append(v)
            prev_x, prev_y = px, py
            
            if total_time >= num_points * dt:
                break

        if total_time == 0.0 or len(raw_x) < 2:
            return [raw_x[0]] * num_points, [raw_y[0]] * num_points, [0.0] * num_points, [raw_v[0]] * num_points

        target_times = [i * dt for i in range(num_points)]

        ref_x = np.interp(target_times, raw_times, raw_x).tolist()
        ref_y = np.interp(target_times, raw_times, raw_y).tolist()
        ref_v = np.interp(target_times, raw_times, raw_v).tolist()

        ref_yaw = []
        for i in range(num_points):
            if i < num_points - 1:
                dx = ref_x[i+1] - ref_x[i]
                dy = ref_y[i+1] - ref_y[i]
            else:
                dx = ref_x[i] - ref_x[i-1]
                dy = ref_y[i] - ref_y[i-1]
            ref_yaw.append(math.atan2(dy, dx))

        return ref_x, ref_y, ref_yaw, ref_v