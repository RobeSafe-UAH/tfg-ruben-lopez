import json
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from PIL import Image
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize

MAP_PATH = "/home/ruben/mvsim_docker/src/rbsf_mvsim/rbsf_mvsim/"
COLORS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']

def load_config(config_file):
    with open(config_file, 'r') as f:
        return json.load(f)

def load_metrics(filepath):
    with open(filepath, 'r') as f:
        return json.load(f)

def load_folder_metrics(folder_path):
    lap_files = [f for f in os.listdir(folder_path) if f.startswith('lap') and f.endswith('.json')]
    lap_data_list = []
    for f in lap_files:
        lap_data_list.append(load_metrics(os.path.join(folder_path, f)))
        
    if not lap_data_list:
        return None, None
        
    valid_laps = [lap for lap in lap_data_list if lap.get('lap_time', float('inf')) > 0]
    if valid_laps:
        valid_laps.sort(key=lambda x: x.get('lap_time', float('inf')))
        fastest_lap = valid_laps[0]
    else:
        fastest_lap = lap_data_list[0]
        
    return lap_data_list, fastest_lap

def find_closest_index(metrics, target_x, target_y):
    coords = np.array([(m['car_x'], m['car_y']) for m in metrics])
    target = np.array([target_x, target_y])
    distances = np.linalg.norm(coords - target, axis=1)
    return np.argmin(distances)

def extract_segment(metrics, start_pt, end_pt):
    if not metrics: return [], [], []
    start_idx = find_closest_index(metrics, start_pt[0], start_pt[1])
    end_idx = find_closest_index(metrics, end_pt[0], end_pt[1])
    
    if start_idx > end_idx:
        end_idx = len(metrics) - 1
        
    segment = metrics[start_idx:end_idx+1]
    if not segment:
        return [], [], []
        
    distances = [0.0]
    accumulated_dist = 0.0
    for i in range(1, len(segment)):
        dx = segment[i]['car_x'] - segment[i-1]['car_x']
        dy = segment[i]['car_y'] - segment[i-1]['car_y']
        accumulated_dist += np.sqrt(dx**2 + dy**2)
        distances.append(accumulated_dist)
        
    cte = [m.get('cte', 0) for m in segment]
    speed_error = [m.get('speed_error', 0) for m in segment]
    return distances, cte, speed_error

def get_lap_progress(metrics):
    if not metrics: return []
    dists = [0.0]
    acc = 0.0
    for i in range(1, len(metrics)):
        dx = metrics[i]['car_x'] - metrics[i-1]['car_x']
        dy = metrics[i]['car_y'] - metrics[i-1]['car_y']
        acc += np.sqrt(dx**2 + dy**2)
        dists.append(acc)
    total_dist = dists[-1] if dists[-1] > 0 else 1.0
    return [(d / total_dist) * 100.0 for d in dists]

def get_centerline_data(csv_path):
    if not os.path.exists(csv_path):
        return None, None, None, None
    data = np.genfromtxt(csv_path, delimiter=',', skip_header=1)
    x = data[:, 0]
    y = data[:, 1]
    
    dists = [0.0]
    acc = 0.0
    for i in range(1, len(x)):
        dx_i = x[i] - x[i-1]
        dy_i = y[i] - y[i-1]
        acc += np.sqrt(dx_i**2 + dy_i**2)
        dists.append(acc)
    total_dist = dists[-1] if dists[-1] > 0 else 1.0
    progress = np.array(dists) / total_dist * 100.0
    
    smooth_w = 10
    x_smooth = np.convolve(x, np.ones(smooth_w)/smooth_w, mode='valid')
    y_smooth = np.convolve(y, np.ones(smooth_w)/smooth_w, mode='valid')
    
    dx = np.gradient(x_smooth)
    dy = np.gradient(y_smooth)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    
    k = np.abs(dx * ddy - dy * ddx) / np.maximum(dx**2 + dy**2, 1e-6)**1.5
    
    pad_left = (len(x) - len(k)) // 2
    pad_right = len(x) - len(k) - pad_left
    k = np.pad(k, (pad_left, pad_right), mode='edge')
    
    k = np.clip(k, 0, np.percentile(k, 95))
    return x, y, progress, k

def plot_performance_summary(all_laps_dict):
    labels = list(all_laps_dict.keys())
    mean_times, std_times = [], []
    mean_ctes, std_ctes = [], []
    mean_speeds, std_speeds = [], []
    
    for label in labels:
        laps = all_laps_dict[label]
        
        times = [lap.get('lap_time', 0) for lap in laps if lap.get('lap_time', 0) > 0]
        mean_times.append(np.mean(times) if times else 0)
        std_times.append(np.std(times) if times else 0)
        
        lap_ctes = []
        lap_speed_errs = []
        for lap in laps:
            metrics = lap.get('lap_metrics', [])
            if metrics:
                lap_ctes.append(np.mean([abs(m.get('cte', 0)) for m in metrics]))
                lap_speed_errs.append(np.mean([abs(m.get('speed_error', 0)) for m in metrics]))
                
        mean_ctes.append(np.mean(lap_ctes) if lap_ctes else 0)
        std_ctes.append(np.std(lap_ctes) if lap_ctes else 0)
        mean_speeds.append(np.mean(lap_speed_errs) if lap_speed_errs else 0)
        std_speeds.append(np.std(lap_speed_errs) if lap_speed_errs else 0)
            
    fig, axes = plt.subplots(1, 3, figsize=(15, 6))
    # fig.suptitle('Global Performance Summary (Averages across laps)', fontsize=18, fontweight='bold')
    
    x = np.arange(len(labels))
    width = 0.6
    cap_kw = {'capsize': 5, 'elinewidth': 2, 'alpha': 0.7}
    
    axes[0].bar(x, mean_times, width, yerr=std_times, color=COLORS[:len(labels)], edgecolor='black', alpha=0.8, error_kw=cap_kw)
    axes[0].set_title('Lap Time (s)', fontsize=14, fontweight='bold')
    axes[0].set_ylabel('Seconds')
    axes[0].grid(axis='y', linestyle='--', alpha=0.7)
    
    axes[1].bar(x, mean_ctes, width, yerr=std_ctes, color=COLORS[:len(labels)], edgecolor='black', alpha=0.8, error_kw=cap_kw)
    axes[1].set_title('Average Absolute CTE (m)', fontsize=14, fontweight='bold')
    axes[1].set_ylabel('Meters')
    axes[1].grid(axis='y', linestyle='--', alpha=0.7)
    
    axes[2].bar(x, mean_speeds, width, yerr=std_speeds, color=COLORS[:len(labels)], edgecolor='black', alpha=0.8, error_kw=cap_kw)
    axes[2].set_title('Average Speed Error (m/s)', fontsize=14, fontweight='bold')
    axes[2].set_ylabel('m/s')
    axes[2].grid(axis='y', linestyle='--', alpha=0.7)
    
    for ax in axes:
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=30, ha='right', fontsize=12)
        
    plt.tight_layout()
    plt.show(block=False)

def plot_segment_comparison(config, all_laps_dict):
    segments = [s['name'] for s in config.get('segments', [])]
    if not segments: return
        
    labels = list(all_laps_dict.keys())
    cte_means = {label: [] for label in labels}
    cte_stds = {label: [] for label in labels}
    speed_means = {label: [] for label in labels}
    speed_stds = {label: [] for label in labels}
    
    for seg_idx, segment in enumerate(config.get('segments', [])):
        for label in labels:
            laps = all_laps_dict[label]
            seg_ctes, seg_speeds = [], []
            for lap in laps:
                _, ctes, speed_errs = extract_segment(lap.get('lap_metrics', []), segment['start'], segment['end'])
                if ctes:
                    seg_ctes.append(np.mean([abs(c) for c in ctes]))
                    seg_speeds.append(np.mean([abs(s) for s in speed_errs]))
            cte_means[label].append(np.mean(seg_ctes) if seg_ctes else 0)
            cte_stds[label].append(np.std(seg_ctes) if seg_ctes else 0)
            speed_means[label].append(np.mean(seg_speeds) if seg_speeds else 0)
            speed_stds[label].append(np.std(seg_speeds) if seg_speeds else 0)
                
    fig, axes = plt.subplots(2, 1, figsize=(14, 10))
    # fig.suptitle('Segment Analysis (Averages across laps)', fontsize=18, fontweight='bold')
    
    x = np.arange(len(segments))
    width = 0.8 / len(labels)
    cap_kw = {'capsize': 3, 'elinewidth': 1.5, 'alpha': 0.6}
    
    for i, label in enumerate(labels):
        offset = (i - len(labels)/2.0 + 0.5) * width
        axes[0].bar(x + offset, cte_means[label], width, yerr=cte_stds[label], label=label, color=COLORS[i % len(COLORS)], edgecolor='black', alpha=0.8, error_kw=cap_kw)
        axes[1].bar(x + offset, speed_means[label], width, yerr=speed_stds[label], label=label, color=COLORS[i % len(COLORS)], edgecolor='black', alpha=0.8, error_kw=cap_kw)
        
    axes[0].set_title('Average Absolute CTE by Segment', fontsize=14, fontweight='bold')
    axes[0].set_ylabel('CTE (m)')
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(segments, fontsize=12)
    axes[0].legend(fontsize=11)
    axes[0].grid(axis='y', linestyle='--', alpha=0.7)
    
    axes[1].set_title('Average Speed Error by Segment', fontsize=14, fontweight='bold')
    axes[1].set_ylabel('Speed Error (m/s)')
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(segments, fontsize=12)
    axes[1].legend(fontsize=11)
    axes[1].grid(axis='y', linestyle='--', alpha=0.7)
    
    plt.tight_layout()
    plt.show(block=False)

def plot_global_errors_vs_progress(config, fastest_laps_dict):
    c_x, c_y, c_prog, c_k = get_centerline_data(config.get('centerline_path', ''))
    has_curvature = c_prog is not None
    
    if has_curvature:
        fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True, gridspec_kw={'height_ratios': [1, 0.08, 1, 0.08]})
        ax_cte = axes[0]
        ax_curv1 = axes[1]
        ax_spd = axes[2]
        ax_curv2 = axes[3]
    else:
        fig, axes = plt.subplots(2, 1, figsize=(14, 10), sharex=True)
        ax_cte = axes[0]
        ax_spd = axes[1]
        ax_curv1 = None
        ax_curv2 = None
        
    # fig.suptitle('Errors vs Lap Progress (Fastest Lap)', fontsize=18, fontweight='bold')
    
    all_speed_errs_after_5 = []
    
    for idx, (label, lap) in enumerate(fastest_laps_dict.items()):
        metrics = lap.get('lap_metrics', [])
        if not metrics: continue
        progress = get_lap_progress(metrics)
        ctes = [m.get('cte', 0) for m in metrics]
        speed_errs = [m.get('speed_error', 0) for m in metrics]
        
        color = COLORS[idx % len(COLORS)]
        ax_cte.plot(progress, ctes, label=label, color=color, linewidth=1.5, alpha=0.7)
        ax_spd.plot(progress, speed_errs, label=label, color=color, linewidth=1.5, alpha=0.7)
        
        for p, s in zip(progress, speed_errs):
            if p >= 5.0:
                all_speed_errs_after_5.append(s)
        
    ax_cte.set_title('Cross-Track Error (CTE)', fontsize=14, fontweight='bold')
    ax_cte.set_ylabel('CTE (m)', fontsize=12)
    ax_cte.grid(True, linestyle='--', alpha=0.6)
    ax_cte.axhline(0, color='black', linewidth=1.5, linestyle=':')
    
    
    ax_spd.set_title('Speed Error', fontsize=14, fontweight='bold')
    ax_spd.set_ylabel('Speed Error (m/s)', fontsize=12)
    ax_spd.grid(True, linestyle='--', alpha=0.6)
    ax_spd.axhline(0, color='black', linewidth=1.5, linestyle=':')
    ax_spd.legend(loc='upper right', fontsize=11, shadow=True)
    
    if all_speed_errs_after_5:
        min_s = min(all_speed_errs_after_5)
        max_s = max(all_speed_errs_after_5)
        margin = (max_s - min_s) * 0.1 if max_s != min_s else 0.1
        ax_spd.set_ylim(min_s - margin, max_s + margin)
    
    if has_curvature:
        Z = c_k.reshape(1, -1)
        ax_curv1.imshow(Z, extent=[0, 100, 0, 1], aspect='auto', cmap='plasma', origin='lower')
        ax_curv1.set_yticks([])
        
        ax_curv2.imshow(Z, extent=[0, 100, 0, 1], aspect='auto', cmap='plasma', origin='lower')
        ax_curv2.set_yticks([])
        ax_curv2.set_xlabel('Lap Progress (%)', fontsize=14)
        ax_curv2.set_xlim(0, 100)
    else:
        ax_spd.set_xlabel('Lap Progress (%)', fontsize=14)
        ax_spd.set_xlim(0, 100)
    
    # Use h_pad to bring the ribbons closer to the plots
    plt.tight_layout(h_pad=0.2)
    plt.show(block=False)

def plot_error_distributions(all_laps_dict):
    labels = list(all_laps_dict.keys())
    cte_data = []
    speed_data = []
    
    valid_labels = []
    for label in labels:
        laps = all_laps_dict[label]
        all_metrics = []
        for lap in laps:
            all_metrics.extend(lap.get('lap_metrics', []))
            
        if all_metrics:
            cte_data.append([m.get('cte', 0) for m in all_metrics])
            speed_data.append([m.get('speed_error', 0) for m in all_metrics])
            valid_labels.append(label)
            
    if not cte_data: return
        
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    # fig.suptitle('Error Distributions across all laps (Stability)', fontsize=18, fontweight='bold')
    
    parts_cte = axes[0].violinplot(cte_data, showmeans=True, showextrema=False)
    axes[0].set_title('Cross-Track Error Distribution', fontsize=14, fontweight='bold')
    axes[0].set_ylabel('CTE (m)', fontsize=12)
    axes[0].axhline(0, color='black', linewidth=1.5, linestyle=':')
    
    parts_speed = axes[1].violinplot(speed_data, showmeans=True, showextrema=False)
    axes[1].set_title('Speed Error Distribution', fontsize=14, fontweight='bold')
    axes[1].set_ylabel('Speed Error (m/s)', fontsize=12)
    axes[1].axhline(0, color='black', linewidth=1.5, linestyle=':')
    
    all_cte = [val for group in cte_data for val in group]
    all_spd = [val for group in speed_data for val in group]
    
    if all_cte:
        p1, p99 = np.percentile(all_cte, [0.5, 99.5])
        margin = (p99 - p1) * 0.1
        axes[0].set_ylim(p1 - margin, p99 + margin)
        
    if all_spd:
        p1, p99 = np.percentile(all_spd, [0.5, 99.5])
        margin = (p99 - p1) * 0.1
        axes[1].set_ylim(p1 - margin, p99 + margin)
    
    for i, ax in enumerate(axes):
        ax.set_xticks(np.arange(1, len(valid_labels) + 1))
        ax.set_xticklabels(valid_labels, rotation=30, ha='right', fontsize=12)
        ax.grid(axis='y', linestyle='--', alpha=0.6)
        
        parts = parts_cte if i == 0 else parts_speed
        for pc_idx, pc in enumerate(parts['bodies']):
            pc.set_facecolor(COLORS[pc_idx % len(COLORS)])
            pc.set_edgecolor('black')
            pc.set_alpha(0.7)
            
        parts['cmeans'].set_color('black')
        
    plt.tight_layout()
    plt.show(block=False)

def setup_map_plot(config, ax):
    map_image_name = config.get('map_image')
    map_resolution = config.get('map_resolution')
    map_origin_raw = config.get('map_origin')
    map_origin = (map_origin_raw[0], map_origin_raw[1]) if map_origin_raw else None

    if not map_image_name: return False

    map_image_path = os.path.join(MAP_PATH, map_image_name)
    if not os.path.exists(map_image_path): return False
        
    pil_image = Image.open(map_image_path)
    if pil_image.mode == 'RGBA':
        white_bg = Image.new('RGB', pil_image.size, (255, 255, 255))
        white_bg.paste(pil_image, mask=pil_image.split()[3])
        pil_image = white_bg
    elif pil_image.mode != 'RGB':
        pil_image = pil_image.convert('RGB')
    
    map_image = np.array(pil_image)
    if map_resolution:
        extent = [0, map_image.shape[1] * map_resolution, map_image.shape[0] * map_resolution, 0]
    else:
        extent = [0, map_image.shape[1], map_image.shape[0], 0]
    
    if map_origin:
        extent[0] += map_origin[0]
        extent[1] += map_origin[0]
        extent[2] -= map_image.shape[0] * map_resolution + map_origin[1]
        extent[3] -= map_image.shape[0] * map_resolution + map_origin[1]
        
    ax.imshow(map_image, extent=extent, origin='upper')
    return True

def plot_full_curvature_map(config):
    c_x, c_y, c_prog, c_k = get_centerline_data(config.get('centerline_path', ''))
    if c_x is None:
        print("Centerline not found, skipping map curvature plot.")
        return
        
    fig, ax = plt.subplots(figsize=(12, 10))
    # fig.suptitle('Circuit Map: Centerline Curvature', fontsize=18, fontweight='bold')
    
    has_map = setup_map_plot(config, ax)
    
    points = np.array([c_x, -c_y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    
    norm = Normalize(vmin=0, vmax=c_k.max())
    lc = LineCollection(segments, cmap='plasma', norm=norm, linewidth=12, alpha=0.9, capstyle='round')
    lc.set_array(c_k[:-1])
    ax.add_collection(lc)
    
    if not has_map:
        ax.set_xlim(c_x.min() - 5, c_x.max() + 5)
        ax.set_ylim(-c_y.max() - 5, -c_y.min() + 5)
        
    cbar = plt.colorbar(lc, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label('Curvature (1/m)', fontsize=14, fontweight='bold')
    cbar.ax.tick_params(labelsize=12)
    
    ax.set_xlabel('X (m)', fontsize=14)
    ax.set_ylabel('Y (m)', fontsize=14)
    ax.grid(True, alpha=0.3, linestyle='--')
    
    plt.tight_layout()
    plt.show(block=False)

def main():
    if len(sys.argv) < 2:
        print("Usage: python advanced_visualizer.py <config_path.json>")
        sys.exit(1)
        
    config_path = sys.argv[1]
    config = load_config(config_path)
    base_path = config.get('metrics_path', '')
    
    all_laps_dict = {}
    fastest_laps_dict = {}
    
    source_list = config.get('folders', config.get('files', []))
    
    for i, item in enumerate(source_list):
        folder_name = item.get('folder', item.get('filename', '').replace('.json', ''))
        if folder_name.endswith('.json'):
            continue
            
        folder_path = os.path.join(base_path, folder_name)
        if not os.path.exists(folder_path):
            print(f"Warning: Folder {folder_path} not found.")
            continue
            
        label = item['label']
        laps, fastest = load_folder_metrics(folder_path)
        
        if laps:
            all_laps_dict[label] = laps
            fastest_laps_dict[label] = fastest
                
    plt.rcParams.update({'font.size': 12, 'axes.labelsize': 14, 'axes.titlesize': 16})
    plt.style.use('bmh') 
    
    if not all_laps_dict:
        print("No valid folder data loaded. Ensure your config uses 'folders' instead of 'files'.")
        sys.exit(1)
        
    print("Generating comprehensive controller comparison plots...")
    
    plot_performance_summary(all_laps_dict)
    
    if config.get('segments'):
        plot_segment_comparison(config, all_laps_dict)
        
    plot_full_curvature_map(config)
    plot_global_errors_vs_progress(config, fastest_laps_dict)
    plot_error_distributions(all_laps_dict)
        
    plt.show()

if __name__ == "__main__":
    main()
