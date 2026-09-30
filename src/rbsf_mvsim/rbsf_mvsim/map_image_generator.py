import xml.etree.ElementTree as ET
from PIL import Image, ImageDraw
import sys
import os

def generate_map_from_xml(xml_file, resolution=0.05):
    """
    Generate an occupancy map image and YAML metadata from an XML file.

    Args:
        xml_file (str): Path to the input XML file.
        resolution (float): Map resolution in meters per pixel.
    """
    tree = ET.parse(xml_file)
    root = tree.getroot()

    lines = []
    min_x, max_x = float('inf'), float('-inf')
    min_y, max_y = float('inf'), float('-inf')

    # Extract wall coordinates from XML
    for el in root.findall(".//element[@class='vertical_plane']"):
        x0 = float(el.find('x0').text)
        y0 = float(el.find('y0').text)
        x1 = float(el.find('x1').text)
        y1 = float(el.find('y1').text)
        
        lines.append((x0, y0, x1, y1))
        
        min_x = min(min_x, x0, x1)
        max_x = max(max_x, x0, x1)
        min_y = min(min_y, y0, y1)
        max_y = max(max_y, y0, y1)

    # Get initial vehicle position from XML
    init_x, init_y = -39.8089, -23.5941 # Safe fallback
    init_pose_el = root.find(".//vehicle/init_pose")
    if init_pose_el is not None and init_pose_el.text:
        parts = init_pose_el.text.strip().split()
        if len(parts) >= 2:
            init_x, init_y = float(parts[0]), float(parts[1])

    # Add 10 meters of margin around the circuit
    pad = 10.0 
    min_x -= pad; max_x += pad
    min_y -= pad; max_y += pad

    # Calculate resolution and pixel size
    width = int((max_x - min_x) / resolution)
    height = int((max_y - min_y) / resolution)

    # Generate image with black background (all occupied by default)
    img = Image.new('L', (width, height), 0)
    draw = ImageDraw.Draw(img)

    def world_to_pixel(x, y):
        """
        Convert world coordinates to pixel coordinates.

        Args:
            x (float): World X coordinate.
            y (float): World Y coordinate.

        Returns:
            tuple: Pixel coordinates (px, py).
        """
        px = int((x - min_x) / resolution)
        py = int(height - (y - min_y) / resolution)
        return px, py

    # Draw white walls to seal the circuit interior
    for (x0, y0, x1, y1) in lines:
        px0, py0 = world_to_pixel(x0, y0)
        px1, py1 = world_to_pixel(x1, y1)
        draw.line((px0, py0, px1, py1), fill=255, width=4)

    # Fill the entire interior with white
    px_start, py_start = world_to_pixel(init_x, init_y)
    ImageDraw.floodfill(img, (px_start, py_start), value=255)

    # Redraw walls in black to define exact boundaries
    for (x0, y0, x1, y1) in lines:
        px0, py0 = world_to_pixel(x0, y0)
        px1, py1 = world_to_pixel(x1, y1)
        draw.line((px0, py0, px1, py1), fill=0, width=3)

    img.save('map_result.png')

    # Build YAML metadata
    yaml_content = f"""image: map_result.png
resolution: {resolution}
origin: [{min_x}, {min_y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    with open('map_result.yaml', 'w') as f:
        f.write(yaml_content)
        
    print("Map and YAML generated successfully.")

if __name__ == "__main__":
    # Get xml filename as parameter
    if len(sys.argv) != 2:
        print("Usage: python map_image_generator.py <file.xml>")
        sys.exit(1)

    xml_file = sys.argv[1]
    generate_map_from_xml(xml_file)
