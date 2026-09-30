#!/usr/bin/env python3

"""Node for automated iterative testing of the simulation."""

import os
import sys
import yaml
import glob
import time
import json
import subprocess
import signal
import shutil
import argparse
from typing import List, Dict, Any

# Path configuration
WORKSPACE_DIR = "/sim_ws"
PARAMS_FILE = os.path.join(WORKSPACE_DIR, "src/rbsf_mvsim/rbsf_mvsim/nodes/params.yaml")
METRICS_DIR = os.path.join(WORKSPACE_DIR, "src/rbsf_mvsim/rbsf_mvsim/metrics")

def load_params() -> Dict[str, Any]:
    """
    Loads parameters from the YAML file.

    Returns:
        Dict[str, Any]: The loaded parameters.
    """
    if not os.path.exists(PARAMS_FILE):
        return {}
    with open(PARAMS_FILE, "r") as f:
        return yaml.safe_load(f) or {}

def get_current_metric_files() -> set:
    """
    Returns a set of the current JSON files in the root metrics directory.

    Returns:
        set: Set of file paths.
    """
    return set(glob.glob(os.path.join(METRICS_DIR, "lap_*.json")))

def setup_output_directory(controller_name: str) -> str:
    """
    Creates and returns the destination folder path, handling duplicates (e.g., test_pid_2).

    Args:
        controller_name (str): The name of the controller being tested.

    Returns:
        str: The path to the created output directory.
    """
    base_folder = os.path.join(METRICS_DIR, f"test_{controller_name}")
    output_dir = base_folder
    counter = 2
    
    while os.path.exists(output_dir):
        output_dir = f"{base_folder}_{counter}"
        counter += 1
        
    os.makedirs(output_dir)
    return output_dir

def run_simulation(timeout_sec: float, baseline_metrics: set) -> str:
    """
    Starts the simulation and returns the path to the new metrics file, or None if timeout/failure.

    Args:
        timeout_sec (float): Timeout in seconds.
        baseline_metrics (set): Set of pre-existing metric files.

    Returns:
        str: Path to the new metrics file, or None.
    """
    cmd = (
        "cd /sim_ws && "
        "source install/local_setup.bash && "
        "ros2 launch rbsf_mvsim rbsf_plan_control.launch.py world_file:=src/rbsf_mvsim/rbsf_mvsim/f1tenth_catalunya.world.xml"
    )
    
    process = subprocess.Popen(
        cmd, 
        shell=True, 
        executable="/bin/bash",
        preexec_fn=os.setsid,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

    start_time = time.time()
    new_metric_file = None
    
    try:
        while time.time() - start_time < timeout_sec:
            current_metrics = get_current_metric_files()
            new_files = list(current_metrics - baseline_metrics)
            
            if new_files:
                new_files.sort(key=os.path.getmtime, reverse=True)
                new_metric_file = new_files[0]
                time.sleep(0.5) # Margin to ensure JSON is written to disk
                break
                
            if process.poll() is not None:
                break # Process terminated unexpectedly
                
            time.sleep(1.0)
    finally:
        # Stop the simulation
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=5.0)
        except Exception:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except Exception:
                pass
        time.sleep(3.0) # Release DDS ports
        
    return new_metric_file

def evaluate_and_store_metric(file_path: str, iter_num: int, output_dir: str) -> bool:
    """
    Evaluates if the lap is valid. If it is, moves it; if not, deletes it. Returns True if valid.

    Args:
        file_path (str): Path to the metrics file.
        iter_num (int): Iteration number.
        output_dir (str): Destination directory for valid metrics.

    Returns:
        bool: True if the lap is valid, False otherwise.
    """
    if not file_path or not os.path.exists(file_path):
        return False
        
    try:
        with open(file_path, "r") as f:
            data = json.load(f)
            
        lap_time = data.get("lap_time", 999.0)
        
        # Invalid lap (off track) or failed
        if lap_time >= 499.0:
            os.remove(file_path)
            return False
            
        # Valid lap: rename and move to destination folder
        target_path = os.path.join(output_dir, f"lap{iter_num}.json")
        shutil.move(file_path, target_path)
        return True
        
    except Exception as e:
        print(f"  [!] Error evaluating file {file_path}: {e}")
        if os.path.exists(file_path):
            os.remove(file_path)
        return False

def compute_and_save_averages(output_dir: str, total_iterations: int, success_count: int):
    """
    Calculates the global averages of all valid JSONs and generates test_averages.json.

    Args:
        output_dir (str): Directory containing the valid metrics files.
        total_iterations (int): Total number of attempted iterations.
        success_count (int): Number of successful iterations.
    """
    all_lap_times = []
    all_speeds = []
    all_ctes = []
    all_speed_errs = []
    
    valid_files = glob.glob(os.path.join(output_dir, "lap*.json"))
    
    for file_path in valid_files:
        with open(file_path, "r") as f:
            data = json.load(f)
            
            all_lap_times.append(data["lap_time"])
            for m in data.get("lap_metrics", []):
                all_speeds.append(m.get("car_v", 0.0))
                all_ctes.append(abs(m.get("cte", 0.0)))
                all_speed_errs.append(abs(m.get("speed_error", 0.0)))

    # Statistical calculations
    success_rate = (success_count / total_iterations) * 100.0 if total_iterations > 0 else 0.0
    mean_lap_time = sum(all_lap_times) / len(all_lap_times) if all_lap_times else 0.0
    mean_speed = sum(all_speeds) / len(all_speeds) if all_speeds else 0.0
    mean_cte = sum(all_ctes) / len(all_ctes) if all_ctes else 0.0
    mean_speed_err = sum(all_speed_errs) / len(all_speed_errs) if all_speed_errs else 0.0

    averages_data = {
        "success_rate": success_rate,
        "lap_time": mean_lap_time,
        "mean_speed": mean_speed,
        "lateral_error": mean_cte,
        "speed_error": mean_speed_err
    }

    output_file = os.path.join(output_dir, "test_averages.json")
    with open(output_file, "w") as f:
        json.dump(averages_data, f, indent=4)
        
    print(f"\nSummary generated at: {output_file}")
    print(json.dumps(averages_data, indent=4))

def main():
    """
    Main function for running automated iterative testing.
    """
    parser = argparse.ArgumentParser(description="Automated testing in ROS 2 simulator.")
    parser.add_argument("-n", "--iterations", type=int, default=10, help="Number of iterations to execute.")
    args = parser.parse_args()

    params = load_params()
    # Extract key variables from params.yaml
    controller_name = params.get("controller_type", "default")
    timeout_sec = params.get("auto_tuner", {}).get("workflow", {}).get("lap_timeout_sec", 300.0)

    output_dir = setup_output_directory(controller_name)
    print(f"Results folder: {output_dir}")
    print(f"Requested iterations: {args.iterations}")
    print("-" * 50)

    success_count = 0

    for i in range(1, args.iterations + 1):
        print(f"Iteration {i}/{args.iterations} started...")
        baseline_metrics = get_current_metric_files()
        
        new_file = run_simulation(timeout_sec, baseline_metrics)
        
        is_valid = evaluate_and_store_metric(new_file, i, output_dir)
        
        if is_valid:
            print(f"  -> VALID lap. File saved as lap{i}.json")
            success_count += 1
        else:
            print("  -> INVALID lap or Timeout. File discarded.")
            
    print("-" * 50)
    print("Iterative execution finished. Processing results...")
    
    compute_and_save_averages(output_dir, args.iterations, success_count)

if __name__ == "__main__":
    main()