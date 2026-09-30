import os
import csv

class MapTools:
    """Tools for loading and handling map centerlines."""

    def __init__(self):
        """Initializes MapTools with default file paths."""
        self.file_path = '/sim_ws/src/rbsf_mvsim/rbsf_mvsim/algorithms/centerline.csv'
        self.refined_file_path = '/sim_ws/src/rbsf_mvsim/rbsf_mvsim/algorithms/centerline_refined.csv'

    def get_manual_centerline(self):
        """Loads a list of (x, y) tuples representing the centerline from a CSV file.

        Returns:
            list: List of (x, y) tuples. Returns [(0.0, 0.0)] if no valid data is found.
        """
        waypoints = []
        selected_file = self.refined_file_path if os.path.exists(self.refined_file_path) else self.file_path
        
        if not os.path.exists(selected_file):
            print(f"WARNING: File {selected_file} not found")
            return [(0.0, 0.0)]

        with open(selected_file, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if row:
                    try:
                        x = float(row[0])
                        y = float(row[1])
                        waypoints.append((x, y))
                    except ValueError:
                        pass

        return waypoints if waypoints else [(0.0, 0.0)]