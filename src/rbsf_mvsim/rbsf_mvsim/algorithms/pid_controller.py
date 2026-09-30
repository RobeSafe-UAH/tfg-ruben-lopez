import math
from rbsf_mvsim.algorithms.base_controller import BaseSteeringController

class PIDController(BaseSteeringController):
    """Pure PID controller for path tracking.

    Calculates the Cross-Track Error (CTE) and applies P, I, D correction.
    """

    def __init__(self, kp, ki, kd, max_steer=0.4):
        """Initializes the PIDController.

        Args:
            kp (float): Proportional gain.
            ki (float): Integral gain.
            kd (float): Derivative gain.
            max_steer (float): Maximum allowed steering angle in radians.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.MAX_STEER = max_steer
        
        self.prev_error = 0.0
        self.integral_error = 0.0

    @classmethod
    def from_params(cls, params: dict):
        """Creates a PID instance from a parameter dictionary.

        Args:
            params (dict): Dictionary with parameters.

        Returns:
            PIDController: Configured instance.
        """
        return cls(
            kp=params.get('kp', 1.0),
            ki=params.get('ki', 0.0),
            kd=params.get('kd', 0.1),
            max_steer=params.get('max_steer', 0.4),
        )

    def calculate_control(self, car_x, car_y, car_yaw, car_speed, path_points, speed_profile):
        """Calculates the steering angle using PID on the Cross-Track Error.

        Args:
            car_x (float): X-coordinate of the car.
            car_y (float): Y-coordinate of the car.
            car_yaw (float): Car's orientation in radians.
            car_speed (float): Current speed of the car.
            path_points (list): List of path points.
            speed_profile (list): Target speed profile.

        Returns:
            tuple:
                - float: Target steering angle in radians.
                - float: Target speed for the closest point.
        """
        if not path_points:
            return 0.0, 0.0

        min_index = self._find_closest_index(car_x, car_y, path_points)
        target_point = path_points[min_index]
        
        try:
            tx, ty = target_point.pose.position.x, target_point.pose.position.y
        except AttributeError:
            tx, ty = target_point[0], target_point[1]

        dx = tx - car_x
        dy = ty - car_y
        distance_error = math.hypot(dx, dy)

        car_vector_x = math.cos(car_yaw)
        car_vector_y = math.sin(car_yaw)
        
        cross_product = car_vector_x * dy - car_vector_y * dx
        
        if cross_product > 0:
            error = distance_error
        else:
            error = -distance_error

        p_term = self.kp * error
        
        self.integral_error += error
        i_term = self.ki * self.integral_error
        
        derivative = error - self.prev_error
        d_term = self.kd * derivative

        steering_angle = p_term + i_term + d_term
        steering_angle = max(-self.MAX_STEER, min(self.MAX_STEER, steering_angle))

        self.prev_error = error

        target_speed = speed_profile[min_index] if min_index < len(speed_profile) else 0.0

        return steering_angle, target_speed