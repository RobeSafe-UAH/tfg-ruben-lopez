import math
from rbsf_mvsim.algorithms.base_controller import BaseSteeringController


class MAPController(BaseSteeringController):
    """Model and Acceleration-based Pursuit (MAP) controller.
    
    Based on the ETH paper. Calculates the required lateral acceleration 
    and maps it to the steering angle.
    """

    def __init__(self, wheelbase, k_lookahead=0.5, min_lookahead=1.0,
                 max_lookahead=10.0, understeer_gradient=0.0, max_steer=0.4):
        """Initializes the MAP Controller.

        Args:
            wheelbase (float): Length of the car (wheelbase).
            k_lookahead (float): Speed gain for lookahead distance.
            min_lookahead (float): Minimum lookahead distance.
            max_lookahead (float): Maximum lookahead distance.
            understeer_gradient (float): Understeer gradient. If 0, it acts almost kinematically.
            max_steer (float): Maximum steering angle in radians.
        """
        self.wheelbase = wheelbase
        self.understeer_gradient = understeer_gradient

        self.k_lookahead = k_lookahead
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead

        self.max_steer = max_steer

    @classmethod
    def from_params(cls, params: dict):
        """Creates a MAPController instance from a parameter dictionary.

        Args:
            params (dict): Dictionary with parameters.

        Returns:
            MAPController: Configured instance.
        """
        return cls(
            max_steer=params.get('max_steer', 0.4),
            wheelbase=params.get('wheelbase', 0.33),
            k_lookahead=params.get('k_lookahead', 0.5),
            min_lookahead=params.get('min_lookahead', 1.0),
            max_lookahead=params.get('max_lookahead', 10.0),
            understeer_gradient=params.get('understeer_gradient', 0.0),
        )

    def get_lookahead_distance(self, current_speed):
        """Calculates the lookahead distance based on the current speed.

        Args:
            current_speed (float): Current speed of the car.

        Returns:
            float: Lookahead distance bounded by min and max limits.
        """
        lookahead = self.k_lookahead * current_speed
        return max(self.min_lookahead, min(self.max_lookahead, lookahead))

    def inverse_vehicle_model(self, lat_accel, speed):
        """Inverse mapping: Converts lateral acceleration and speed into steering angle.
        
        Uses a Single-Track Steady-State vehicle model.

        Args:
            lat_accel (float): Required lateral acceleration.
            speed (float): Current speed.

        Returns:
            float: Required steering angle.
        """
        if speed < 0.5:
            return math.atan(self.wheelbase * lat_accel / (0.5 ** 2))

        delta_kinematic = math.atan(self.wheelbase * lat_accel / (speed ** 2))
        delta_dynamic = self.understeer_gradient * lat_accel

        return delta_kinematic + delta_dynamic

    def calculate_control(self, car_x, car_y, car_yaw, current_speed, path_points, speed_profile):
        """Calculates the steering angle using MAP.

        Args:
            car_x (float): X-coordinate of the car.
            car_y (float): Y-coordinate of the car.
            car_yaw (float): Car's orientation in radians.
            current_speed (float): Current speed of the car in m/s.
            path_points (list): List of path points.
            speed_profile (list): List of target speeds for each path point.

        Returns:
            tuple:
                - float: Target steering angle in radians.
                - float: Target speed for the closest point.
        """
        if not path_points:
            return 0.0, 0.0

        lookahead_dist = self.get_lookahead_distance(current_speed)

        min_index = self._find_closest_index(car_x, car_y, path_points)
        tx, ty = self._find_target_point(car_x, car_y, path_points, min_index, lookahead_dist)

        dx = tx - car_x
        dy = ty - car_y

        local_x = dx * math.cos(car_yaw) + dy * math.sin(car_yaw)
        local_y = -dx * math.sin(car_yaw) + dy * math.cos(car_yaw)

        distance_sq = local_x ** 2 + local_y ** 2

        if distance_sq < 0.001:
            return 0.0, speed_profile[min_index] if min_index < len(speed_profile) else 0.0

        lat_accel_req = (2.0 * (current_speed ** 2) * local_y) / distance_sq

        steering_angle = self.inverse_vehicle_model(lat_accel_req, current_speed)
        steering_angle = max(-self.max_steer, min(self.max_steer, steering_angle))

        target_speed = speed_profile[min_index] if min_index < len(speed_profile) else 0.0

        return steering_angle, target_speed
