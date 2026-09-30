#!/usr/bin/env python3

import sys
import yaml
import os
import argparse
import numpy as np
import matplotlib.pyplot as plt

def load_history(filepath):
    with open(filepath, 'r') as f:
        return yaml.safe_load(f)

def plot_score_evolution(history_data):
    history = history_data['tuning_history']
    iterations = [entry['iteration'] for entry in history]
    current_scores = [entry['current_score'] for entry in history]
    best_scores = [entry['best_score'] for entry in history]
    
    valid_scores = [s for s in current_scores if s < 400]
    
    plt.figure(figsize=(12, 7))
    
    plt.scatter(iterations, current_scores, color='#1f77b4', alpha=0.6, s=60, edgecolors='w', label='Current Score')
    plt.plot(iterations, current_scores, color='#1f77b4', alpha=0.3, linewidth=1)
    plt.plot(iterations, best_scores, color='#d62728', marker='o', linewidth=3, markersize=8, label='Best Score (Convergence)')
    
    plt.ylabel('Score', fontsize=14)
        
    if valid_scores:
        y_max = max(valid_scores) * 1.1
        y_min = min(best_scores) * 0.9 if min(best_scores) > 0 else 0
        plt.ylim(bottom=y_min, top=y_max)
        
        crashes_x = [i for i, s in zip(iterations, current_scores) if s >= 400]
        if crashes_x:
            plt.scatter(crashes_x, [y_max]*len(crashes_x), color='black', marker='X', s=120, zorder=5, clip_on=False, label='Crash / Timeout / Invalid')
    
    plt.xlabel('Optimization Iteration', fontsize=14)
    # plt.title('Bayesian Optimization: Score Evolution Over Time', fontsize=18, fontweight='bold', pad=15)
    plt.legend(fontsize=12, framealpha=0.9, shadow=True)
    plt.grid(True, which='both', linestyle='--', alpha=0.6)
    plt.xticks(fontsize=12)
    plt.yticks(fontsize=12)
    plt.tight_layout()
    plt.show(block=False)

def plot_parameter_evolution(history_data):
    metadata = history_data.get('metadata', {})
    tunable_keys = metadata.get('tunable_keys', [])
    bounds = metadata.get('bounds', {})
    history = history_data.get('tuning_history', [])
    
    if not tunable_keys:
        return
        
    iterations = [entry['iteration'] for entry in history]
    
    n_params = len(tunable_keys)
    cols = min(2, n_params)
    rows = int(np.ceil(n_params / cols))
    
    fig, axes = plt.subplots(rows, cols, figsize=(14, 5 * rows))
    if n_params == 1:
        axes = np.array([axes])
    axes = axes.flatten()
    
    for i, key in enumerate(tunable_keys):
        ax = axes[i]
        vals = [entry['current_params'][key] for entry in history]
        best_vals = [entry['best_params'][key] for entry in history]
        
        ax.plot(iterations, vals, marker='o', linestyle='-', color='#1f77b4', 
                alpha=0.7, linewidth=2, markersize=7, label='Evaluated Value')
        ax.plot(iterations, best_vals, marker='', linestyle='--', color='#d62728', 
                linewidth=3, label='Current Best')
        
        if key in bounds:
            low = bounds[key]['low']
            high = bounds[key]['high']
            ax.axhline(low, color='black', linestyle=':', alpha=0.7, linewidth=2, label='Bounds' if i==0 else "")
            ax.axhline(high, color='black', linestyle=':', alpha=0.7, linewidth=2)
            margin = (high - low) * 0.15
            if margin == 0:
                margin = 0.1
            ax.set_ylim(low - margin, high + margin)
            
        ax.set_title(f'Evolution of {key}', fontsize=16, fontweight='bold', pad=10)
        ax.set_ylabel('Parameter Value', fontsize=14)
        ax.tick_params(axis='both', which='major', labelsize=12)
        ax.grid(True, linestyle='--', alpha=0.7)
        
        if i == 0:
            ax.legend(loc='best', fontsize=12, framealpha=0.9, shadow=True)
            
    # Hide unused subplots
    for ax in axes[n_params:]:
        ax.set_visible(False)
        
    for i in range(cols):
        idx = min(n_params - 1, len(axes) - cols + i)
        if idx >= 0:
            axes[idx].set_xlabel('Iteration', fontsize=14)
            
    # fig.suptitle('Weight Evolution over Time (Bayesian Optimization)', fontsize=22, fontweight='bold', y=0.98)
    plt.tight_layout()
    fig.subplots_adjust(top=0.90 + (0.02 if rows == 1 else 0))
    plt.show(block=False)

def plot_parameter_vs_score(history_data):
    metadata = history_data.get('metadata', {})
    tunable_keys = metadata.get('tunable_keys', [])
    history = history_data.get('tuning_history', [])
    
    if not tunable_keys:
        return
        
    current_scores = [entry['current_score'] for entry in history]
    valid_scores = [s for s in current_scores if s < 400]
    y_max = max(valid_scores) * 1.1 if valid_scores else max(current_scores)
    
    n_params = len(tunable_keys)
    cols = min(3, n_params)
    rows = int(np.ceil(n_params / cols))
    
    fig, axes = plt.subplots(rows, cols, figsize=(12, 3.5 * rows))
    if n_params == 1:
        axes = np.array([axes])
    axes = axes.flatten()
    
    iterations = [entry['iteration'] for entry in history]
    use_log = max(current_scores) > 100 and min(current_scores) < 100
    
    for i, key in enumerate(tunable_keys):
        ax = axes[i]
        vals = [entry['current_params'][key] for entry in history]
        
        sc = ax.scatter(vals, current_scores, c=iterations, cmap='viridis', alpha=0.8, edgecolors='none')
        valid_vals = [v for v, s in zip(vals, current_scores) if s < 400]
        valid_sc = [s for s in current_scores if s < 400]
        valid_iters = [it for it, s in zip(iterations, current_scores) if s < 400]
        
        crash_vals = [v for v, s in zip(vals, current_scores) if s >= 400]
        
        if valid_vals:
            sc = ax.scatter(valid_vals, valid_sc, c=valid_iters, cmap='viridis', alpha=0.8, edgecolors='none')
            
        if crash_vals:
            ax.scatter(crash_vals, [y_max]*len(crash_vals), color='red', marker='X', s=80, alpha=0.6, clip_on=False)
            
        ax.set_ylim(top=y_max)
        ax.set_title(key)
        ax.set_xlabel('Parameter Value')
        if use_log:
            ax.set_ylabel('Score (Log Scale)')
            ax.set_yscale('log')
        else:
            ax.set_ylabel('Score')
        ax.set_ylabel('Score')
        ax.grid(True, alpha=0.3)
        
    # Hide unused subplots
    for ax in axes[n_params:]:
        ax.set_visible(False)
        
    plt.tight_layout()
    
    # Add colorbar globally (adjusting layout to fit it)
    fig.subplots_adjust(right=0.9)
    cbar_ax = fig.add_axes([0.92, 0.15, 0.02, 0.7])
    fig.colorbar(sc, cax=cbar_ax, label='Iteration (dark=early, bright=late)')
    if valid_scores:
        fig.colorbar(sc, cax=cbar_ax, label='Iteration (dark=early, bright=late)')
    
    # fig.suptitle('Parameter Landscape (Value vs Score)', fontsize=16, y=0.98)
    plt.show(block=False)

def plot_parallel_coordinates(history_data):
    """
    Plots a parallel coordinates graph to show combinations of parameters and their scores.
    """
    metadata = history_data.get('metadata', {})
    tunable_keys = metadata.get('tunable_keys', [])
    history = history_data.get('tuning_history', [])
    
    if len(tunable_keys) < 2:
        return
        
    # Extract data
    data = []
    scores = []
    
    for entry in history:
        row = [entry['current_params'][k] for k in tunable_keys]
        data.append(row)
        scores.append(entry['current_score'])
        
    data = np.array(data)
    scores = np.array(scores)
    
    # Normalize data for parallel coordinates
    data_norm = np.zeros_like(data)
    for i in range(len(tunable_keys)):
        col_min = np.min(data[:, i])
        col_max = np.max(data[:, i])
        if col_max > col_min:
            data_norm[:, i] = (data[:, i] - col_min) / (col_max - col_min)
        else:
            data_norm[:, i] = 0.5
            
    fig, ax = plt.subplots(figsize=(12, 6))
    
    valid_mask = scores < 400
    
    if np.any(valid_mask):
        valid_scores = scores[valid_mask]
        cmap = plt.cm.get_cmap('coolwarm_r') 
        norm = plt.Normalize(np.min(valid_scores), np.max(valid_scores))
        
        # Plot crashes first as faint lines
        for i in range(len(data_norm)):
            if not valid_mask[i]:
                ax.plot(range(len(tunable_keys)), data_norm[i, :], 
                        color='black', alpha=0.3, linewidth=1.0, linestyle=':')
        
        # Plot valid scores colored by performance
        for i in range(len(data_norm)):
            if valid_mask[i]:
                ax.plot(range(len(tunable_keys)), data_norm[i, :], 
                        color=cmap(norm(scores[i])), alpha=0.7, linewidth=1.5)
                        
        # Plot best in thick green
        best_idx = np.argmin(scores)
        ax.plot(range(len(tunable_keys)), data_norm[best_idx, :], 
                color='lime', alpha=1.0, linewidth=3, label='Best Config')
                
        sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
        sm.set_array([])
        fig.colorbar(sm, ax=ax, label='Score (Successful Laps)')
    else:
        # Fallback if all are crashes
        for i in range(len(data_norm)):
            ax.plot(range(len(tunable_keys)), data_norm[i, :], 
                    color='black', alpha=0.5, linewidth=1.0)
                    
    # Invisible line for crash legend
    if not np.all(valid_mask):
        ax.plot([], [], color='black', alpha=0.5, linestyle=':', label='Crash/Timeout')
        
    ax.set_xticks(range(len(tunable_keys)))
    ax.set_xticklabels(tunable_keys, rotation=15, ha='right')
    ax.set_ylabel('Normalized Parameter Value')
    # ax.set_title('Parallel Coordinates: Parameter Combinations vs Score')
    ax.grid(True, axis='x', color='black', alpha=0.3, linewidth=1.5)
    ax.legend(loc='upper right')
    
    plt.tight_layout()
    plt.show(block=False)
            

def main():
    parser = argparse.ArgumentParser(description='Visualize Bayesian Optimization Tuning History.')
    parser.add_argument('-f', '--file', type=str, 
                        default='/sim_ws/src/rbsf_mvsim/rbsf_mvsim/nodes/tuning_history.yaml',
                        help='Path to the tuning_history.yaml file.')
    
    args = parser.parse_args()
    
    # Handle path translation if running outside docker directly on the host workspace
    filepath = args.file
    if filepath.startswith('/sim_ws/') and not os.path.exists(filepath):
        # Try local path mapping if we are in host environment
        local_ws = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
        alt_path = filepath.replace('/sim_ws/src/rbsf_mvsim/rbsf_mvsim', local_ws)
        if os.path.exists(alt_path):
            filepath = alt_path
            
    if not os.path.exists(filepath):
        print(f"Error: Could not find {filepath}")
        print("Please provide a valid path using -f <path>")
        sys.exit(1)
        
    print(f"Loading data from {filepath}...")
    history_data = load_history(filepath)
    
    if 'tuning_history' not in history_data or not history_data['tuning_history']:
        print("Error: No 'tuning_history' data found in the provided YAML file.")
        sys.exit(1)
        
    print(f"Loaded {len(history_data['tuning_history'])} iterations of history.")
    print("Generating plots (close all windows to exit)...")
    
    plot_score_evolution(history_data)
    plot_parameter_evolution(history_data)
    plot_parameter_vs_score(history_data)
    plot_parallel_coordinates(history_data)
    
    plt.show() # Blocking show to keep all windows open

if __name__ == '__main__':
    main()
