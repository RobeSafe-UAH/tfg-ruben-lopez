import numpy as np

def yaw2quaternion(yaw):
    """
        Función que convierte un ángulo yaw a cuaternión.
    """
    return [0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0)]

def quaternion2yaw(quat):
    """
        Función que convierte un cuaternion a ángulo
    """
    return np.arctan2(2.0 * (quat.w * quat.z + quat.x * quat.y), 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))

