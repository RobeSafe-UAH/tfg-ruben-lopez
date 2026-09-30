import argparse
import csv
import os
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt, maximum_filter
from scipy.interpolate import CubicSpline


def load_sim_map_yaml(sim_config_path: str) -> str:
    """
    Loads the map path from the simulation configuration YAML.

    Args:
        sim_config_path (str): The path to the simulation config YAML.

    Returns:
        str: The full path to the map YAML file.

    Raises:
        ValueError: If the map path is not found in the configuration.
    """
    with open(sim_config_path, "r", encoding="utf-8") as f:
        sim_cfg = yaml.safe_load(f) or {}

    ros_params = sim_cfg.get("bridge", {}).get("ros__parameters", {})
    map_path = ros_params.get("map_path", "")
    if not map_path:
        raise ValueError(f"bridge.ros__parameters.map_path not found in {sim_config_path}")

    return map_path + ".yaml"


def load_occupancy_from_map_yaml(map_yaml_path: str) -> Tuple[np.ndarray, float, Tuple[float, float]]:
    """
    Loads occupancy grid information from the map YAML file.

    Args:
        map_yaml_path (str): The path to the map YAML file.

    Returns:
        Tuple[np.ndarray, float, Tuple[float, float]]: A tuple containing the 
            occupancy array, resolution, and origin (x, y).

    Raises:
        ValueError: If 'image' path is missing from the YAML file.
    """
    with open(map_yaml_path, "r", encoding="utf-8") as f:
        map_cfg = yaml.safe_load(f) or {}

    image_rel = map_cfg.get("image", "")
    if not image_rel:
        raise ValueError(f"'image' not found in {map_yaml_path}")

    image_path = image_rel
    if not os.path.isabs(image_rel):
        image_path = os.path.join(os.path.dirname(map_yaml_path), image_rel)

    resolution = float(map_cfg.get("resolution", 0.05))
    origin_raw = map_cfg.get("origin", [0.0, 0.0, 0.0])
    origin = (float(origin_raw[0]), float(origin_raw[1]))

    raw = np.array(Image.open(image_path).convert("L"), dtype=np.uint8)
    negate = int(map_cfg.get("negate", 0))
    occupied_thresh = float(map_cfg.get("occupied_thresh", 0.65))

    if negate == 0:
        occ_prob = (255.0 - raw.astype(np.float32)) / 255.0
    else:
        occ_prob = raw.astype(np.float32) / 255.0

    occupied = occ_prob > occupied_thresh
    return occupied, resolution, origin


def load_centerline_csv(csv_path: str) -> List[Tuple[float, float]]:
    """
    Loads a centerline from a CSV file.

    Args:
        csv_path (str): The path to the CSV file.

    Returns:
        List[Tuple[float, float]]: A list of (x, y) coordinates representing the centerline.

    Raises:
        ValueError: If no valid points are found in the CSV.
    """
    points: List[Tuple[float, float]] = []
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) < 2:
                continue
            try:
                x = float(row[0])
                y = float(row[1])
                points.append((x, y))
            except ValueError:
                continue

    if not points:
        raise ValueError(f"No valid points found in {csv_path}")

    return points


def world_to_grid(x: float, y: float, resolution: float, origin: Tuple[float, float], shape: Tuple[int, int]) -> Tuple[int, int]:
    """
    Converts world coordinates to grid indices.

    Args:
        x (float): X coordinate in the world frame.
        y (float): Y coordinate in the world frame.
        resolution (float): The resolution of the map.
        origin (Tuple[float, float]): The origin (x, y) of the map.
        shape (Tuple[int, int]): The shape (height, width) of the grid.

    Returns:
        Tuple[int, int]: Grid coordinates as (row, column).
    """
    mx = int((x - origin[0]) / resolution)
    my = int((y - origin[1]) / resolution)
    h, w = shape
    col = mx
    row = (h - 1) - my
    return row, col


def grid_to_world(row: int, col: int, resolution: float, origin: Tuple[float, float], shape: Tuple[int, int]) -> Tuple[float, float]:
    """
    Converts grid indices to world coordinates.

    Args:
        row (int): The row index in the grid.
        col (int): The column index in the grid.
        resolution (float): The resolution of the map.
        origin (Tuple[float, float]): The origin (x, y) of the map.
        shape (Tuple[int, int]): The shape (height, width) of the grid.

    Returns:
        Tuple[float, float]: World coordinates as (x, y).
    """
    h, _ = shape
    mx = col
    my = (h - 1) - row
    x = origin[0] + (mx + 0.5) * resolution
    y = origin[1] + (my + 0.5) * resolution
    return x, y


def clamp_cell(row: int, col: int, shape: Tuple[int, int]) -> Tuple[int, int]:
    """
    Clamps cell coordinates to ensure they fall within the grid boundaries.

    Args:
        row (int): The target row index.
        col (int): The target column index.
        shape (Tuple[int, int]): The shape (height, width) of the grid.

    Returns:
        Tuple[int, int]: The clamped (row, col).
    """
    h, w = shape
    row = max(0, min(h - 1, row))
    col = max(0, min(w - 1, col))
    return row, col


def refine_point_to_clearance_peak(
    row: int,
    col: int,
    dist_map: np.ndarray,
    search_radius_cells: int,
    anchor_weight: float,
) -> Tuple[int, int]:
    """
    Refines a point to the nearest local peak in the clearance distance map.

    Args:
        row (int): Starting row index.
        col (int): Starting column index.
        dist_map (np.ndarray): Distance map representing clearance from obstacles.
        search_radius_cells (int): Search radius around the point.
        anchor_weight (float): Penalty weight for moving away from the start point.

    Returns:
        Tuple[int, int]: Refined (row, col) coordinates.
    """
    h, w = dist_map.shape
    r0 = max(0, row - search_radius_cells)
    r1 = min(h, row + search_radius_cells + 1)
    c0 = max(0, col - search_radius_cells)
    c1 = min(w, col + search_radius_cells + 1)

    best_r, best_c = row, col
    best_score = -1e18

    for rr in range(r0, r1):
        for cc in range(c0, c1):
            clear_score = float(dist_map[rr, cc])
            dr = rr - row
            dc = cc - col
            anchor_penalty = anchor_weight * float(dr * dr + dc * dc)
            score = clear_score - anchor_penalty
            if score > best_score:
                best_score = score
                best_r, best_c = rr, cc

    return best_r, best_c


def compute_ridge_mask(
    dist_map: np.ndarray,
    free_mask: np.ndarray,
    ridge_window: int,
    min_clearance_cells: float,
) -> np.ndarray:
    """
    Approximates the medial axis: local maxima in the free space distance map.

    Args:
        dist_map (np.ndarray): The distance map.
        free_mask (np.ndarray): Boolean mask of free space cells.
        ridge_window (int): The window size to compute local maxima.
        min_clearance_cells (float): Minimum required clearance in cells.

    Returns:
        np.ndarray: A boolean mask where true values represent the medial ridge.
    """
    w = max(3, int(ridge_window))
    if w % 2 == 0:
        w += 1
    local_max = dist_map >= maximum_filter(dist_map, size=w)
    return free_mask & local_max & (dist_map >= min_clearance_cells)


def refine_point_to_ridge(
    row: int,
    col: int,
    dist_map: np.ndarray,
    ridge_mask: np.ndarray,
    search_radius_cells: int,
    anchor_weight: float,
    prev_cell: Optional[Tuple[int, int]],
    continuity_weight: float,
    tangent_vec: Optional[Tuple[float, float]],
    along_penalty_weight: float,
) -> Tuple[int, int]:
    """
    Refines a point towards the medial ridge, factoring in continuity and tangency penalties.

    Args:
        row (int): Starting row index.
        col (int): Starting column index.
        dist_map (np.ndarray): The clearance distance map.
        ridge_mask (np.ndarray): The boolean medial ridge mask.
        search_radius_cells (int): Search radius limit in cells.
        anchor_weight (float): Penalty weight for moving away from the anchor.
        prev_cell (Optional[Tuple[int, int]]): The previous refined cell.
        continuity_weight (float): Penalty weight to enforce continuity.
        tangent_vec (Optional[Tuple[float, float]]): The local tangent vector of the path.
        along_penalty_weight (float): Penalty weight for moving along the path direction.

    Returns:
        Tuple[int, int]: The refined (row, col) indices.
    """
    h, w = dist_map.shape
    r0 = max(0, row - search_radius_cells)
    r1 = min(h, row + search_radius_cells + 1)
    c0 = max(0, col - search_radius_cells)
    c1 = min(w, col + search_radius_cells + 1)

    best_r, best_c = row, col
    best_score = -1e18
    found_ridge = False

    tangent_unit: Optional[np.ndarray] = None
    if tangent_vec is not None:
        tv = np.array([float(tangent_vec[0]), float(tangent_vec[1])], dtype=np.float64)
        nrm = float(np.linalg.norm(tv))
        if nrm > 1e-9:
            tangent_unit = tv / nrm

    for rr in range(r0, r1):
        for cc in range(c0, c1):
            if not ridge_mask[rr, cc]:
                continue
            found_ridge = True
            clear_score = float(dist_map[rr, cc])
            da_r = rr - row
            da_c = cc - col
            anchor_penalty = anchor_weight * float(da_r * da_r + da_c * da_c)

            continuity_penalty = 0.0
            if prev_cell is not None:
                dp_r = rr - prev_cell[0]
                dp_c = cc - prev_cell[1]
                continuity_penalty = continuity_weight * float(dp_r * dp_r + dp_c * dp_c)

            along_penalty = 0.0
            if tangent_unit is not None:
                d = np.array([float(da_r), float(da_c)], dtype=np.float64)
                along = float(np.dot(d, tangent_unit))
                along_penalty = along_penalty_weight * along * along

            score = clear_score - anchor_penalty - continuity_penalty - along_penalty
            if score > best_score:
                best_score = score
                best_r, best_c = rr, cc

    if found_ridge:
        return best_r, best_c

    # Fallback when there is no ridge in the local window.
    return refine_point_to_clearance_peak(
        row,
        col,
        dist_map,
        search_radius_cells=search_radius_cells,
        anchor_weight=anchor_weight,
    )


def refine_cells_with_ridge(
    seed_cells: List[Tuple[int, int]],
    dist_map: np.ndarray,
    ridge_mask: np.ndarray,
    search_radius_cells: int,
    anchor_weight: float,
    continuity_weight: float,
    along_penalty_weight: float,
) -> List[Tuple[int, int]]:
    """
    Refines a list of seed cells towards the medial ridge.

    Args:
        seed_cells (List[Tuple[int, int]]): Original list of seed cells.
        dist_map (np.ndarray): Distance map for clearance.
        ridge_mask (np.ndarray): Precomputed medial ridge mask.
        search_radius_cells (int): Search radius in cells.
        anchor_weight (float): Penalty for moving far from seeds.
        continuity_weight (float): Penalty for discontinuity with the previous point.
        along_penalty_weight (float): Penalty for sliding along the tangent vector.

    Returns:
        List[Tuple[int, int]]: A sequence of refined cell indices.
    """
    n = len(seed_cells)
    tangents: List[Tuple[float, float]] = []
    for i in range(n):
        p_prev = seed_cells[(i - 1) % n]
        p_next = seed_cells[(i + 1) % n]
        tr = float(p_next[0] - p_prev[0])
        tc = float(p_next[1] - p_prev[1])
        tangents.append((tr, tc))

    out: List[Tuple[int, int]] = []
    prev: Optional[Tuple[int, int]] = None
    for i, (r, c) in enumerate(seed_cells):
        rr, cc = refine_point_to_ridge(
            r,
            c,
            dist_map,
            ridge_mask,
            search_radius_cells=search_radius_cells,
            anchor_weight=anchor_weight,
            prev_cell=prev,
            continuity_weight=continuity_weight,
            tangent_vec=tangents[i],
            along_penalty_weight=along_penalty_weight,
        )
        out.append((rr, cc))
        prev = (rr, cc)
    return out


def smooth_closed_path(points: List[Tuple[float, float]], window: int = 7) -> List[Tuple[float, float]]:
    """
    Smooths a closed loop path using a moving average window.

    Args:
        points (List[Tuple[float, float]]): The list of path coordinates.
        window (int, optional): Size of the smoothing window. Defaults to 7.

    Returns:
        List[Tuple[float, float]]: The smoothed path.
    """
    if window < 3 or len(points) < window:
        return points

    arr = np.array(points, dtype=np.float64)
    n = arr.shape[0]
    half = window // 2
    out = np.zeros_like(arr)

    for i in range(n):
        idx = [(i + k) % n for k in range(-half, half + 1)]
        out[i] = arr[idx].mean(axis=0)

    return [(float(x), float(y)) for x, y in out]


def resample_closed_path(points: List[Tuple[float, float]], num_points: int) -> List[Tuple[float, float]]:
    """
    Resamples a closed path to have a specific number of points.

    Args:
        points (List[Tuple[float, float]]): Original path points.
        num_points (int): The target number of points.

    Returns:
        List[Tuple[float, float]]: Resampled path points.
    """
    arr = np.array(points, dtype=np.float64)
    if len(arr) < 3 or num_points <= 2:
        return points

    # Avoid duplicating start/end if the path is already closed.
    if np.linalg.norm(arr[0] - arr[-1]) < 1e-9:
        arr = arr[:-1]
    if len(arr) < 3:
        return points

    # Parameterization by arc length to respect local geometry when densifying.
    arr_closed = np.vstack([arr, arr[0]])
    seg = np.diff(arr_closed, axis=0)
    seg_len = np.sqrt((seg ** 2).sum(axis=1))
    cum = np.concatenate([[0.0], np.cumsum(seg_len)])
    total = float(cum[-1])
    if total <= 1e-9:
        return points

    try:
        spline_x = CubicSpline(cum, arr_closed[:, 0], bc_type="periodic")
        spline_y = CubicSpline(cum, arr_closed[:, 1], bc_type="periodic")
        targets = np.linspace(0.0, total, num_points, endpoint=False)
        xs = spline_x(targets)
        ys = spline_y(targets)
        return [(float(x), float(y)) for x, y in zip(xs, ys)]
    except Exception:
        # Linear fallback if spline creation fails.
        targets = np.linspace(0.0, total, num_points, endpoint=False)
        out: List[Tuple[float, float]] = []
        j = 0
        for t in targets:
            while j < len(seg_len) - 1 and cum[j + 1] < t:
                j += 1
            d0 = cum[j]
            d1 = cum[j + 1]
            alpha = 0.0 if d1 - d0 <= 1e-9 else (t - d0) / (d1 - d0)
            p = arr_closed[j] * (1.0 - alpha) + arr_closed[j + 1] * alpha
            out.append((float(p[0]), float(p[1])))
        return out


def closed_path_length(points: List[Tuple[float, float]]) -> float:
    """
    Calculates the total length of a closed path.

    Args:
        points (List[Tuple[float, float]]): The path coordinates.

    Returns:
        float: The cumulative length of the closed path.
    """
    arr = np.array(points, dtype=np.float64)
    if len(arr) < 2:
        return 0.0
    arr_closed = np.vstack([arr, arr[0]])
    seg = np.diff(arr_closed, axis=0)
    seg_len = np.sqrt((seg ** 2).sum(axis=1))
    return float(seg_len.sum())


def save_centerline_csv(points: List[Tuple[float, float]], out_csv: str) -> None:
    """
    Saves the refined centerline coordinates to a CSV file.

    Args:
        points (List[Tuple[float, float]]): The refined path points.
        out_csv (str): Path where the CSV should be saved.
    """
    out_path = Path(out_csv)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    with out_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        for x, y in points:
            writer.writerow([x, y])


def main() -> None:
    """
    Main function to parse arguments and run the centerline refinement generator.
    """
    parser = argparse.ArgumentParser(description="Generates a refined centerline using the occupancy map.")
    parser.add_argument(
        "--map-yaml",
        default="/sim_ws/src/rbsf_mvsim/rbsf_mvsim/mapa_catalunya.yaml",
        help="Direct path to the static map YAML file",
    )
    parser.add_argument(
        "--input-centerline",
        default="/sim_ws/src/rbsf_mvsim/rbsf_mvsim/algorithms/centerline.csv",
        help="Seed centerline CSV file",
    )
    parser.add_argument(
        "--output-centerline",
        default="/sim_ws/src/rbsf_mvsim/rbsf_mvsim/algorithms/centerline_refined.csv",
        help="Output CSV for the refined centerline",
    )
    parser.add_argument(
        "--search-radius-m",
        type=float,
        default=0.7,
        help="Local search radius (m) to find clearance maximums around each point",
    )
    parser.add_argument(
        "--anchor-weight",
        type=float,
        default=0.015,
        help="Penalty for moving away from the seed point (in cells space)",
    )
    parser.add_argument(
        "--continuity-weight",
        type=float,
        default=0.01,
        help="Penalty to avoid jumps between consecutive points",
    )
    parser.add_argument(
        "--along-penalty-weight",
        type=float,
        default=0.25,
        help="Penalty for moving in the longitudinal direction of the seed path",
    )
    parser.add_argument(
        "--ridge-window",
        type=int,
        default=7,
        help="Window size to detect ridges (approximate medial axis)",
    )
    parser.add_argument(
        "--min-clearance-m",
        type=float,
        default=0.2,
        help="Minimum clearance (m) required to consider a centerline candidate",
    )
    parser.add_argument(
        "--smooth-window",
        type=int,
        default=7,
        help="Circular smoothing window size",
    )
    parser.add_argument(
        "--resample-points",
        type=int,
        default=0,
        help="Final number of points (0 = calculated automatically using --target-spacing-m)",
    )
    parser.add_argument(
        "--target-spacing-m",
        type=float,
        default=0.20,
        help="Target spacing between points during densification (m)",
    )

    args = parser.parse_args()

    occupied, resolution, origin = load_occupancy_from_map_yaml(args.map_yaml)
    free = ~occupied
    dist_map = distance_transform_edt(free)
    min_clearance_cells = max(1e-6, float(args.min_clearance_m) / resolution)
    ridge_mask = compute_ridge_mask(
        dist_map,
        free,
        ridge_window=int(args.ridge_window),
        min_clearance_cells=min_clearance_cells,
    )

    seed_points = load_centerline_csv(args.input_centerline)
    shape = occupied.shape
    search_radius_cells = max(1, int(round(args.search_radius_m / resolution)))

    seed_cells: List[Tuple[int, int]] = []
    for x, y in seed_points:
        r, c = world_to_grid(x, y, resolution, origin, shape)
        r, c = clamp_cell(r, c, shape)
        seed_cells.append((r, c))

    # Dual pass (forward + backward) to reduce lateral bias in curves.
    fwd_cells = refine_cells_with_ridge(
        seed_cells,
        dist_map,
        ridge_mask,
        search_radius_cells=search_radius_cells,
        anchor_weight=float(args.anchor_weight),
        continuity_weight=float(args.continuity_weight),
        along_penalty_weight=float(args.along_penalty_weight),
    )

    rev_seed = list(reversed(seed_cells))
    rev_cells_tmp = refine_cells_with_ridge(
        rev_seed,
        dist_map,
        ridge_mask,
        search_radius_cells=search_radius_cells,
        anchor_weight=float(args.anchor_weight),
        continuity_weight=float(args.continuity_weight),
        along_penalty_weight=float(args.along_penalty_weight),
    )
    rev_cells = list(reversed(rev_cells_tmp))

    refined_cells: List[Tuple[int, int]] = []
    for (rf, cf), (rr, cr) in zip(fwd_cells, rev_cells):
        r = int(round((rf + rr) * 0.5))
        c = int(round((cf + cr) * 0.5))
        refined_cells.append(clamp_cell(r, c, shape))

    refined_points = [grid_to_world(r, c, resolution, origin, shape) for r, c in refined_cells]
    refined_points = smooth_closed_path(refined_points, window=max(3, int(args.smooth_window)))

    if args.resample_points and args.resample_points > 2:
        final_points = int(args.resample_points)
    else:
        spacing = max(0.02, float(args.target_spacing_m))
        length_m = closed_path_length(refined_points)
        auto_points = max(3, int(round(length_m / spacing)))
        # Never reduce below current resolution; only maintain or densify.
        final_points = max(len(refined_points), auto_points)

    if final_points > 2:
        refined_points = resample_closed_path(refined_points, final_points)

    save_centerline_csv(refined_points, args.output_centerline)

    print(f"Refined centerline generated: {args.output_centerline}")
    print(f"Input points: {len(seed_points)} | Output points: {len(refined_points)}")
    print(f"Ridge candidates: {int(np.count_nonzero(ridge_mask))}")


if __name__ == "__main__":
    main()
