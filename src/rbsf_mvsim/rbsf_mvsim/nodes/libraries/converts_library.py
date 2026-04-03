import numpy as np

def yaw2quaternion(yaw):
    """
        Función que convierte un ángulo yaw a cuaternión.
    """
    return [0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0)]

def quaternion2yaw(quaternion):
    """
        Función que convierte un cuaternion a ángulo
    """
    return 0