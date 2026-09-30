import math
import time
import json
import yaml
import os
from typing import Any, Dict, Optional, Tuple

import numpy as np
from PIL import Image

from rbsf_mvsim.algorithms.map_tools import MapTools

class Metrics:
    """Class to compute, store, and manage lap metrics."""

    def __init__(self):
        """Initializes the Metrics object and resets the state."""
        self.map_tools = MapTools()
        self.centerline = self.map_tools.get_manual_centerline()
        self.reset()

    def reset(self):
        """Resets all metrics and internal variables."""
        self.total_cte = 0.0
        self.max_cte = 0.0
        self.total_speed_error = 0.0
        self.samples = 0
        self.start_time = None
        self.lap_count = 0
        self.last_index = 0
        self.path_length_indices = 0
        self.lap_metrics = []
        self.path = "/sim_ws/src/rbsf_mvsim/rbsf_mvsim/metrics/"

        self.map_config_path = "/sim_ws/src/rbsf_mvsim/rbsf_mvsim/Catalunya_map.yaml"
        self.map_image_name = None
        self.map_resolution = None
        self.map_origin = None
        self.lap_armed = False
        self.middle_zone_low_ratio = 0.2
        self.middle_zone_high_ratio = 0.8
        self.motion_start_speed_threshold = 0.2
        
    def update(self, car_x, car_y, car_v, path_points, target_speeds, current_index):
        """Updates metrics for the current moment and accumulates for later averages.

        Args:
            car_x (float): Current car X position.
            car_y (float): Current car Y position.
            car_v (float): Current car speed.
            path_points (list): List of path points (x, y).
            target_speeds (list): Target speeds for each path point.
            current_index (int): Index of the closest path point.

        Returns:
            dict: Current state dictionary or None if there is no path.
        """
        if not path_points:
            return None

        if self.start_time is None:
            if car_v < self.motion_start_speed_threshold:
                self.last_index = current_index
                return {
                    "cte": 0.0,
                    "speed_error": 0.0,
                    "lap_completed": False,
                    "lap_time": 0.0,
                    "lap_invalidated": False,
                    "collision": False,
                }
            self.start_time = time.time()

        self.path_length_indices = len(path_points)

        if self.path_length_indices > 0:
            middle_low = self.path_length_indices * self.middle_zone_low_ratio
            middle_high = self.path_length_indices * self.middle_zone_high_ratio
            if middle_low <= current_index <= middle_high:
                self.lap_armed = True
        
        px, py = path_points[current_index]
        cte = math.hypot(px - car_x, py - car_y)

        self.total_cte += cte
        self.max_cte = max(self.max_cte, cte)
        
        speed_err = 0.0
        if target_speeds and current_index < len(target_speeds):
            tgt_v = target_speeds[current_index]
            speed_err = abs(tgt_v - car_v)
            self.total_speed_error += speed_err
            
        self.samples += 1

        self.lap_metrics.append({
            "car_x": car_x,
            "car_y": car_y,
            "car_v": car_v,
            "cte": cte,
            "speed_error": speed_err,
            "timestamp": time.time() - self.start_time
        })
        
        lap_completed = False
        lap_time = 0.0
        now = time.time()

        centerline_dist = min(math.hypot(cx - car_x, cy - car_y) for cx, cy in self.centerline)
        if centerline_dist > 1.5:
            progress = 0.0
            if self.path_length_indices > 0:
                progress = current_index / self.path_length_indices
            lap_time = 1000.0 - (500.0 * progress)
            lap_completed = True
            
        elif (
            self.start_time is not None
            and self.last_index < self.path_length_indices * 0.1
            and current_index > self.path_length_indices * 0.9
        ):
            # Car crossed start line in reverse
            lap_time = 1000.0
            lap_completed = True
            
        elif (
            self.lap_armed
            and self.last_index > self.path_length_indices * 0.9
            and current_index < self.path_length_indices * 0.1
        ):  
            if self.start_time is not None:
                lap_time = now - self.start_time
                self.lap_count += 1
                lap_completed = True
            self.start_time = now

        if lap_completed:
            with open('/sim_ws/src/rbsf_mvsim/rbsf_mvsim/nodes/params.yaml', 'r') as f:
                params = yaml.safe_load(f)
            params_list = [f"{key}: {value}" for key, value in params.items()]
            comment = " ".join(params_list)

            target_speeds_list = [float(v) for v in target_speeds] if target_speeds else []
            name = f"{self.path}lap_{self.lap_count}_{int(now)}.json"

            with open(name, 'w') as f:
                json.dump({
                    "lap_time": lap_time,
                    "lap_metrics": self.lap_metrics,
                    "path_points": path_points,
                    "target_speeds": target_speeds_list,
                    "map_image": self.map_image_name,
                    "map_resolution": self.map_resolution,
                    "map_origin": self.map_origin,
                    "comment": comment
                }, f, indent=4)
            
            self.lap_metrics = []
            self.lap_armed = False

        self.last_index = current_index
        
        return {
            "cte": cte,
            "speed_error": speed_err,
            "lap_completed": lap_completed,
            "lap_time": lap_time,
        }

    def get_summary(self):
        """Returns the average summary of metrics.

        Returns:
            dict: Summary of metrics.
        """
        if self.samples == 0: return {}
        return {
            "avg_cte": self.total_cte / self.samples,
            "max_cte": self.max_cte,
            "avg_speed_error": self.total_speed_error / self.samples,
            "laps": self.lap_count
        }

    def get_progress(self):
        """Returns current lap progress as a ratio (0.0 to 1.0)."""
        if self.path_length_indices > 0:
            return self.last_index / self.path_length_indices
        return 0.0