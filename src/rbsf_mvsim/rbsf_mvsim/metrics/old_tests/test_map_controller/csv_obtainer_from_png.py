import cv2
import csv
import numpy as np
import argparse
import sys

def extract_contours_from_png(png_path, csv_path, resolution, threshold):
    """
    Extract contours from a PNG image and save them as vector segments in a CSV.
    """
    # 1. Load the image in grayscale
    img = cv2.imread(png_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"Error: Could not load the image: {png_path}")
        sys.exit(1)
    
    height, width = img.shape

    # 2. Binarize the image
    # Assume obstacles/walls are dark (< threshold).
    # THRESH_BINARY_INV inverts colors: obstacles in white (255), background in black (0).
    # This is necessary because findContours looks for white objects.
    _, mask = cv2.threshold(img, threshold, 255, cv2.THRESH_BINARY_INV)

    # 3. Extract contours and approximate them to polygons
    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    
    segments = []

    # 4. Transform to metric coordinates and generate vector segments
    for cnt in contours:
        # Vertex simplification for straight lines
        epsilon = 0.002 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        
        world_pts = []
        for pt in approx:
            u, v = pt[0]
            # Metric transformation assuming the origin (0,0) at the bottom left corner
            x = u * resolution
            y = (height - 1 - v) * resolution
            world_pts.append((x, y))
            
        # Create point pairs (x0, y0) -> (x1, y1)
        for i in range(len(world_pts)):
            p1 = world_pts[i]
            p2 = world_pts[(i + 1) % len(world_pts)]
            
            # Discard submillimeter noise segments
            if np.hypot(p2[0] - p1[0], p2[1] - p1[1]) >= 0.01:
                segments.append([round(p1[0], 4), round(p1[1], 4), round(p2[0], 4), round(p2[1], 4)])

    # 5. Write results to the CSV file
    with open(csv_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['x0', 'y0', 'x1', 'y1'])
        writer.writerows(segments)

    print(f"Extraction completed. {len(segments)} segments saved in: {csv_path}")

def main():
    """
    Main entry point for extracting contours.
    """
    parser = argparse.ArgumentParser(description="Extracts contours from a .png and saves them to a vector .csv.")
    parser.add_argument("--input", required=True, help="Image path (.png)")
    parser.add_argument("--output", required=True, help="Output file path (.csv)")
    parser.add_argument("--res", type=float, default=0.02, help="Resolution in meters/pixel (default: 0.02)")
    parser.add_argument("--thresh", type=int, default=127, help="Darkness threshold for obstacles (0-255)")
    
    args = parser.parse_args()
    
    extract_contours_from_png(args.input, args.output, args.res, args.thresh)

if __name__ == "__main__":
    main()
