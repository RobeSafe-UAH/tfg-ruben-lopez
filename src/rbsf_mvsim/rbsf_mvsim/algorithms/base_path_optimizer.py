import math
from abc import ABC, abstractmethod
import numpy as np

class BasePathOptimizer(ABC):
    """Abstract base class for path optimizers.

    Defines the common interface and provides shared utilities.
    """

    @classmethod
    @abstractmethod
    def from_params(cls, params: dict):
        """Creates an optimizer instance from a parameter dictionary.

        Args:
            params (dict): Dictionary containing the optimizer parameters.

        Returns:
            BasePathOptimizer: A configured instance of the optimizer.
        """
        ...

    @abstractmethod
    def optimize_path(self, path_points):
        """Optimizes the trajectory and generates a target speed profile.

        Args:
            path_points (list): List of tuples (x, y, z) representing the trajectory to optimize.

        Returns:
            tuple:
                - list: Optimized path points as a list of tuples (x, y, z).
                - list: Target speeds for each point in the optimized path.
        """
        ...

    def _extract_coordinates(self, path_points):
        """Extracts (x, y) coordinates for NumPy usage.

        Args:
            path_points (list): List of tuples (x, y, z).

        Returns:
            np.ndarray: An Nx2 array of (x, y) coordinates.
        """
        return np.array([(p[0], p[1]) for p in path_points])
        
    def _rebuild_path(self, optimized_coords):
        """Rebuilds the path points from optimized coordinates.

        Args:
            optimized_coords (np.ndarray): An Nx2 array with the optimized (x, y) coordinates.

        Returns:
            list: A list of tuples (x, y, z) where z is 0.0.
        """
        return [(x, y, 0.0) for x, y in optimized_coords]