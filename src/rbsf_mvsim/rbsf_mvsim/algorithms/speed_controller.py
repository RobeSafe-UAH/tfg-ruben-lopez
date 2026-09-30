class SpeedController:
    """Controls and smooths the target velocity to respect physical limits.
    This algorithm is not in use now, the speed controller is developed for each control algorithm.
    """

    def __init__(self, max_accel, max_decel):
        """Initializes the SpeedController.
        
        Args:
            max_accel (float): Maximum allowed acceleration (m/s^2).
            max_decel (float): Maximum allowed deceleration (m/s^2).
        """
        self.max_accel = max_accel
        self.max_decel = max_decel
        self.last_speed_command = 0.0

    def calculate_velocity(self, current_velocity, target_velocity, dt):
        """Smooths the target velocity respecting the acceleration and deceleration limits.
        
        Args:
            current_velocity (float): Current actual speed of the car (m/s).
            target_velocity (float): Ideal target speed requested by the planner (m/s).
            dt (float): Time elapsed since the last call (seconds).

        Returns:
            float: Smoothed safe velocity to be sent to the motor.
        """
        if dt <= 0.0:
            return current_velocity

        delta_v = target_velocity - self.last_speed_command
        
        if delta_v > 0:
            max_delta_v = self.max_accel * dt
            safe_delta_v = min(delta_v, max_delta_v)
        else:
            max_delta_v = self.max_decel * dt
            safe_delta_v = max(delta_v, -max_delta_v)

        safe_velocity = self.last_speed_command + safe_delta_v
        self.last_speed_command = safe_velocity
        
        return safe_velocity