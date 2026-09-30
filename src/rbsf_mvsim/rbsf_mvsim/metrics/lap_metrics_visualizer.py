import json
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
import numpy as np
from PIL import Image
from datetime import datetime
from matplotlib.collections import LineCollection
import sys


MAP_PATH = "/home/ruben/mvsim_docker/src/rbsf_mvsim/rbsf_mvsim/" # Change to the map path
METRICS_PATH = "/home/ruben/mvsim_docker/src/rbsf_mvsim/rbsf_mvsim/metrics/" # Change to the metrics path

def plot_paths(original_path_points, metrics, map_image_name, map_resolution, map_origin, comment):
    """
    Visualizes the two paths (original and actual) on the background map.
    The original path is shown in black. The actual path is shown in a color scale between blue and red depending on the speed.
    The scale with the corresponding values is shown on the side.
    Args:
        original_path_points: list of tuples (x, y) representing the original path points
        metrics: dictionary with lap metrics for each frame
            car_x, car_y: Car position
            car_v: Car speed
            cte: Cross-track error
            speed_error: Speed error
            timestamp: Time at that frame
        map_image_name: background map image name
        map_resolution: background map image scale (meters per pixel)
        map_origin: position of the 0,0 point in the image
        comment: comment to include in the title
    """

    map_image = mpimg.imread(MAP_PATH + map_image_name)

    # Load with PIL to correctly handle transparency
    pil_image = Image.open(MAP_PATH + map_image_name)
    
    # If it has an alpha channel, convert RGBA to RGB with a white background
    if pil_image.mode == 'RGBA':
        white_background = Image.new('RGB', pil_image.size, (255, 255, 255))
        white_background.paste(pil_image, mask=pil_image.split()[3])  # Use alpha channel as mask
        pil_image = white_background
    elif pil_image.mode != 'RGB':
        pil_image = pil_image.convert('RGB')
    
    # Convert PIL Image to numpy array
    map_image = np.array(pil_image)

    # Create figure and axes
    fig, ax = plt.subplots(figsize=(12, 10))

    # Calculate image scale
    if map_resolution:
        extent = [0, map_image.shape[1] * map_resolution, map_image.shape[0] * map_resolution, 0]
    else:
        extent = [0, map_image.shape[1], map_image.shape[0], 0]
    
    # Adjust origin position
    if map_origin:
        extent[0] += map_origin[0]
        extent[1] += map_origin[0]
        extent[2] -= map_image.shape[0] * map_resolution + map_origin[1]
        extent[3] -= map_image.shape[0] * map_resolution + map_origin[1]
    ax.imshow(map_image, extent=extent, origin='upper')

    # Extract actual path from metrics
    real_x = [m['car_x'] for m in metrics]
    real_y = [-m['car_y'] for m in metrics]
    velocities = [m['car_v'] for m in metrics]

    # Normalize speeds for the color scale
    norm = Normalize(vmin=0, vmax=max(velocities))
    cmap = plt.cm.get_cmap('viridis')
    scalar_map = ScalarMappable(cmap=cmap, norm=norm)

    # Create line segments for the actual path and assign colors based on speed
    segments = []
    colors = []
    for i in range(len(real_x) - 1):
        segments.append([(real_x[i], real_y[i]), (real_x[i+1], real_y[i+1])])
        colors.append(cmap(norm(velocities[i])))
    
    # Draw all lines using LineCollection to improve performance
    line_collection = LineCollection(segments, colors=colors, linewidths=2.5)
    ax.add_collection(line_collection)

    # Original path in black
    if original_path_points:
        original_x = [p[0] for p in original_path_points]
        original_y = [-p[1] for p in original_path_points]
        ax.plot(original_x, original_y, 'k-', linewidth=1, label='Original Path', alpha=0.7)

    # Color scale for speed
    cbar = plt.colorbar(scalar_map, ax=ax, label='Speed (m/s)')

    # Labels and legend
    ax.set_xlabel('X (pixels)')
    ax.set_ylabel('Y (pixels)')
    ax.set_title(f'Path Comparison - {comment}')
    ax.legend()
    ax.grid(True, alpha=0.0)

    plt.tight_layout()    
    plt.show()

def plot_speed_profile(metrics, target_speeds, original_path_points, comment):
    """
    Visualizes the actual car speed over time compared to the target speed profile.
    Args:
        metrics: dictionary with lap metrics for each frame
            car_x, car_y: Car position
            car_v: Car speed
            timestamp: Time at that frame
        target_speeds: list of target speeds for each path point
        original_path_points: list of tuples (x, y) representing the original path points
        comment: comment to include in the title
    """
    timestamps = [m['timestamp'] for m in metrics]
    actual_speeds = [m['car_v'] for m in metrics]
    
    # Calculate target speed for each metric based on the closest path point
    objective_speeds = []
    for m in metrics:
        car_x, car_y = m['car_x'], m['car_y']
        
        # Find the closest path point
        min_distance = float('inf')
        closest_index = 0
        for i, (px, py) in enumerate(original_path_points):
            distance = np.sqrt((px - car_x)**2 + (py - car_y)**2)
            if distance < min_distance:
                min_distance = distance
                closest_index = i
                
        # Get the target speed for that point
        if closest_index < len(target_speeds):
            objective_speeds.append(target_speeds[closest_index])
        else:
            objective_speeds.append(0)

    plt.figure(figsize=(10, 5))
    plt.plot(timestamps, actual_speeds, label='Actual Speed', color='blue')
    if target_speeds:
        plt.plot(timestamps, objective_speeds, label='Target Speed', color='red', linestyle='--')
    
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (m/s)')
    plt.title(f'Speed Profile - {comment}')
    plt.legend()
    plt.grid(True)
    plt.show()


def visualize_lap_metrics(json_path):
    """
    Load JSON metrics and call visualization functions.
    """
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    # PRINT MAIN DATA
    print(f"Visualizing race metrics from: {json_path}")
    comment = data.get('comment', 'No comment available.')
    print(f"Comment: {comment}")
    metrics = data.get('lap_metrics', [])
    avg_cte = 0
    avg_speed_error = 0
    count = 0
    for m in metrics:
        avg_cte += m.get('cte', 0)
        avg_speed_error += m.get('speed_error', 0)
        count += 1
    avg_cte /= count if count > 0 else 1
    avg_speed_error /= count if count > 0 else 1
    print(f"Average cross-track error: {avg_cte:.3f} m")
    print(f"Average speed error: {avg_speed_error:.3f} m/s")
    lap_time = data.get('lap_time', 0)
    print(f"Lap time: {lap_time:.2f} s")

    # TRACK AND PATHS GRAPH
    print("Generating paths graph...")
    original_path_points = data.get('path_points', [])
    map_image_name = data.get('map_image', None)
    map_resolution = data.get('map_resolution', None)
    map_origin_raw = data.get('map_origin', None)
    map_origin = (map_origin_raw[0], map_origin_raw[1]) if map_origin_raw else None

    if map_image_name:
        plot_paths(original_path_points, metrics, map_image_name, map_resolution, map_origin, comment)
    else:
        print("Map image not found for visualization.")

    # SPEEDS AND SPEED PROFILE GRAPH
    print("Generating speeds graph...")
    target_speeds = data.get('target_speeds', [])
    plot_speed_profile(metrics, target_speeds, original_path_points, comment)

    
def main():
    """
    Main function to visualize lap metrics.
    """
    # Get filename as argument
    if len(sys.argv) > 1:
        filename = sys.argv[1]
        visualize_lap_metrics(METRICS_PATH + filename)
    else:
        visualize_lap_metrics(METRICS_PATH + "test.json") # Default value for testing

if __name__ == "__main__":
    main()
