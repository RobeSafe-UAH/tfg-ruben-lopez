import math
from rbsf_mvsim.algorithms.base_controller import BaseSteeringController


class PurePursuit(BaseSteeringController):
    """Simplified version of pure pursuit for steering angle control.

    This algorithm does not calculate speed control.
    """

    def __init__(self, lookahead_distance, max_steer=0.4, wheelbase=0.33):
        """Initializes the PurePursuit controller.

        Args:
            lookahead_distance (float): Minimum distance the car looks ahead.
            max_steer (float): Maximum steering angle in radians.
            wheelbase (float): Distance between axles in meters.
        """
        self.lookahead = lookahead_distance
        self.max_steer = max_steer
        self.wheelbase = wheelbase

    @classmethod
    def from_params(cls, params: dict):
        """Creates a PurePursuit instance from a parameter dictionary.

        Args:
            params (dict): Dictionary containing the parameters.

        Returns:
            PurePursuit: Configured controller instance.
        """
        return cls(
            lookahead_distance=params.get('lookahead_distance', 1.5),
            max_steer=params.get('max_steer', 0.4),
            wheelbase=params.get('wheelbase', 0.33),
        )

    def calculate_control(self, car_x, car_y, car_yaw, car_speed, path_points, speed_profile):
        """Calculates the steering angle needed to follow the path.

        Args:
            car_x (float): X-coordinate of the car.
            car_y (float): Y-coordinate of the car.
            car_yaw (float): Car's orientation in radians.
            car_speed (float): Car's current speed.
            path_points (list): List of path points.
            speed_profile (list): List of target speeds (not used for steering in this controller).

        Returns:
            tuple:
                - float: Target steering angle in radians.
                - float: Target speed for the closest point.
        """
        if not path_points:
            return 0.0, 0.0

        min_index = self._find_closest_index(car_x, car_y, path_points)
        tx, ty = self._find_target_point(car_x, car_y, path_points, min_index, self.lookahead)

        dx = tx - car_x
        dy = ty - car_y

        transformed_y = -dx * math.sin(car_yaw) + dy * math.cos(car_yaw)

        curvature = 2.0 * transformed_y / (self.lookahead ** 2)

        steering_angle = math.atan(curvature * self.wheelbase)

        steering_angle = max(-self.max_steer, min(self.max_steer, steering_angle))

        target_speed = speed_profile[min_index] if min_index < len(speed_profile) else 0.0
        
        return steering_angle, target_speed
