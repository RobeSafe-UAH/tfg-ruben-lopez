import argparse
import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import sys
import os

# ---------------------------------------------------------
# DATA EXTRACTION MODULES
# ---------------------------------------------------------

def extract_physical_points(image_path, resolution, threshold=127):
    """Extracts obstacle points from an image and converts them to meters."""
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"Error: Could not load the image {image_path}")
        sys.exit(1)
        
    y, x = np.where(img < threshold)
    if len(x) == 0:
        print(f"Error: No obstacles found in {image_path}.")
        sys.exit(1)
        
    return np.column_stack((x, y)) * resolution

def extract_points_from_csv(csv_path, sample_dist=0.01):
    """Samples vector segments into a dense point cloud."""
    try:
        data = np.loadtxt(csv_path, delimiter=',', skiprows=1)
        if data.ndim == 1: data = np.atleast_2d(data)
        
        all_points = []
        for row in data:
            x0, y0, x1, y1 = row[0], row[1], row[2], row[3]
            length = np.hypot(x1 - x0, y1 - y0)
            if length == 0:
                all_points.append([x0, y0])
                continue
            
            num_samples = max(2, int(length / sample_dist))
            xs = np.linspace(x0, x1, num_samples)
            ys = np.linspace(y0, y1, num_samples)
            all_points.append(np.column_stack((xs, ys)))
            
        return np.vstack(all_points)
    except Exception as e:
        print(f"Error processing CSV: {e}")
        sys.exit(1)

# ---------------------------------------------------------
# ALIGNMENT AND METRIC MODULES
# ---------------------------------------------------------

def icp_2d(pts_source, pts_target, max_iterations=50, tolerance=1e-5):
    """Spatially aligns the source cloud to the target minimizing the error."""
    src = np.copy(pts_source)
    trg = np.copy(pts_target)
    
    # Centroid pre-alignment
    src -= np.mean(src, axis=0)
    trg_centroid = np.mean(trg, axis=0)
    src += trg_centroid
    
    tree = KDTree(trg)
    prev_error = float('inf')
    
    for _ in range(max_iterations):
        distances, indices = tree.query(src)
        matched_trg = trg[indices]
        
        centroid_src = np.mean(src, axis=0)
        centroid_trg = np.mean(matched_trg, axis=0)
        
        src_centered = src - centroid_src
        trg_centered = matched_trg - centroid_trg
        
        H = np.dot(src_centered.T, trg_centered)
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)
        
        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1
            R = np.dot(Vt.T, U.T)
            
        t = centroid_trg - np.dot(R, centroid_src)
        src = np.dot(src, R.T) + t
        
        mean_error = np.mean(distances)
        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error
        
    return src

def calculate_chamfer_distance(pts1, pts2):
    """Calculates the mean bidirectional distance between two point clouds."""
    tree1, tree2 = KDTree(pts1), KDTree(pts2)
    d1_to_2, _ = tree2.query(pts1)
    d2_to_1, _ = tree1.query(pts2)
    return np.mean(d1_to_2) + np.mean(d2_to_1)

# ---------------------------------------------------------
# REPORTING AND VISUALIZATION MODULES
# ---------------------------------------------------------

def print_metrics_table(results):
    """Prints the results of the calculated distances in the terminal."""
    best_algo = min(results, key=lambda x: x['distance'])
    
    print("\n" + "="*60)
    print(f"{'Algorithm':<30} | {'Chamfer Distance (m)':<25}")
    print("-" * 60)
    for res in results:
        marker = " <--- BEST" if res['name'] == best_algo['name'] else ""
        print(f"{res['name']:<30} | {res['distance']:.6f}{marker}")
    print("="*60 + "\n")

def plot_maps(pts_ref, processed_slams):
    """Generates a 2D plot with the reference and SLAM maps overlaid."""
    plt.figure(figsize=(12, 10))
    
    # Plot Reference
    plt.scatter(pts_ref[:, 0], pts_ref[:, 1], c='black', s=2, label='Ground Truth', alpha=0.7)
    
    # Plot each SLAM algorithm
    for slam in processed_slams:
        plt.scatter(slam['points'][:, 0], slam['points'][:, 1], 
                    c=slam['color'], s=1, label=f"{slam['name']} ({slam['distance']:.4f} m)", alpha=0.6)

    # Format
    plt.title('ICP Alignment and Chamfer Distance')
    plt.xlabel('X Coordinate (meters)')
    plt.ylabel('Y Coordinate (meters)')
    plt.legend(markerscale=5)
    plt.axis('equal')
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.show()

# ---------------------------------------------------------
# MAIN ORCHESTRATOR
# ---------------------------------------------------------

def main():
    """
    Main entry point to execute the full evaluation pipeline.
    """
    parser = argparse.ArgumentParser(description="Complete pipeline: ICP -> Chamfer -> Plot.")
    parser.add_argument("--slam", nargs='+', required=True, help="List of SLAM maps (.pgm)")
    parser.add_argument("--ref_img", help="Path to reference map (.png)")
    parser.add_argument("--ref_csv", help="Path to reference map (.csv)")
    parser.add_argument("--res_ref", type=float, help="Reference resolution (m/px).")
    parser.add_argument("--thresh", type=int, default=127, help="Threshold for obstacles")
    args = parser.parse_args()

    RES_SLAM = 0.02
    colors = ['red', 'blue', 'green', 'orange', 'purple', 'cyan', 'magenta']
    
    # 1. Load and prepare Reference
    if args.ref_csv:
        pts_ref = extract_points_from_csv(args.ref_csv, sample_dist=0.01) 
    else:
        pts_ref = extract_physical_points(args.ref_img, args.res_ref, args.thresh)
    
    # Center reference at origin as ICP base
    pts_ref -= np.mean(pts_ref, axis=0)

    # 2. Process each SLAM map
    results = []
    for idx, slam_file in enumerate(args.slam):
        algo_name = os.path.basename(slam_file)
        algo_name = algo_name.split('.')[0]
        pts_slam = extract_physical_points(slam_file, RES_SLAM, args.thresh)

        pts_interm = np.zeros_like(pts_slam)
        
        # A. Fine alignment via ICP
        pts_slam_aligned = icp_2d(pts_slam, pts_ref)
        
        # B. Metric calculation after alignment
        distance = calculate_chamfer_distance(pts_slam_aligned, pts_ref)

        # C. Store results
        results.append({
            'name': algo_name,
            'points': pts_slam_aligned,
            'distance': distance,
            'color': colors[idx % len(colors)]
        })

    # 3. Terminal Report
    print_metrics_table(results)
    
    # 4. Visualization
    plot_maps(pts_ref, results)

if __name__ == "__main__":
    main()