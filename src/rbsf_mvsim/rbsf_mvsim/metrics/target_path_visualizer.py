import json
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
import numpy as np
from PIL import Image
from matplotlib.collections import LineCollection
import sys


MAP_PATH = "/home/ruben/mvsim_docker/src/rbsf_mvsim/rbsf_mvsim/"
METRICS_PATH = "/home/ruben/mvsim_docker/src/rbsf_mvsim/rbsf_mvsim/metrics/"

def plot_target_path(original_path_points, target_speeds, map_image_name, map_resolution, map_origin):
    """
    Visualizes the target path on the background map, colored by the target speed.
    """
    if not map_image_name:
        print("Map image not found for visualization.")
        return

    # Load image
    map_image = mpimg.imread(MAP_PATH + map_image_name)
    pil_image = Image.open(MAP_PATH + map_image_name)
    
    # If it has an alpha channel, convert RGBA to RGB with a white background
    if pil_image.mode == 'RGBA':
        white_background = Image.new('RGB', pil_image.size, (255, 255, 255))
        white_background.paste(pil_image, mask=pil_image.split()[3])
        pil_image = white_background
    elif pil_image.mode != 'RGB':
        pil_image = pil_image.convert('RGB')
    
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

    if original_path_points and target_speeds:
        original_x = [p[0] for p in original_path_points]
        original_y = [-p[1] for p in original_path_points]
        
        # Ensure we have target speeds for the segments
        num_points = min(len(original_x), len(target_speeds))
        
        norm = Normalize(vmin=min(target_speeds), vmax=max(target_speeds))
        cmap = plt.cm.get_cmap('viridis')
        scalar_map = ScalarMappable(cmap=cmap, norm=norm)

        segments = []
        colors = []
        for i in range(num_points - 1):
            segments.append([(original_x[i], original_y[i]), (original_x[i+1], original_y[i+1])])
            colors.append(cmap(norm(target_speeds[i])))

        line_collection = LineCollection(segments, colors=colors, linewidths=2.5)
        ax.add_collection(line_collection)

        cbar = plt.colorbar(scalar_map, ax=ax, label='Target Speed (m/s)')
    elif original_path_points:
        # Fallback if no target_speeds are available
        original_x = [p[0] for p in original_path_points]
        original_y = [-p[1] for p in original_path_points]
        ax.plot(original_x, original_y, 'k-', linewidth=2.5, label='Target Path')
        ax.legend()
        print("No target speeds found, drawing path in black.")
    
    # Labels
    ax.set_xlabel('X (pixels)')
    ax.set_ylabel('Y (pixels)')
    ax.set_title('Optimized path speed')
    ax.grid(True, alpha=0.0)

    plt.tight_layout()    
    plt.show()

def visualize_target_path(json_path):
    """
    Load JSON metrics and call visualization function.
    """
    try:
        with open(json_path, 'r') as f:
            data = json.load(f)
    except Exception as e:
        print(f"Error loading {json_path}: {e}")
        return
        
    print(f"Visualizing target path from: {json_path}")
    
    original_path_points = data.get('path_points', [])
    target_speeds = data.get('target_speeds', [])
    map_image_name = data.get('map_image', None)
    map_resolution = data.get('map_resolution', None)
    map_origin_raw = data.get('map_origin', None)
    map_origin = (map_origin_raw[0], map_origin_raw[1]) if map_origin_raw else None

    plot_target_path(original_path_points, target_speeds, map_image_name, map_resolution, map_origin)

def main():
    """
    Main function to visualize target path.
    """
    if len(sys.argv) > 1:
        filename = sys.argv[1]
        visualize_target_path(METRICS_PATH + filename)
    else:
        visualize_target_path(METRICS_PATH + "test.json") # Default value for testing

if __name__ == "__main__":
    main()

