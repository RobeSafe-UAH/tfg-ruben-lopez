#!/usr/bin/env python3

"""Node for Bayesian hyperparameter tuning of the simulation."""

import os
import sys
import yaml
import glob
import time
import json
import subprocess
import signal
from typing import Any, Dict, List

# Attempt to import the Bayesian Optimization library, if not, install it
try:
    from skopt import gp_minimize
    from skopt.space import Real
except ImportError:
    print("scikit-optimize is not installed. Installing it now...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "scikit-optimize"])
    from skopt import gp_minimize
    from skopt.space import Real

# Paths within the container / workspace
WORKSPACE_DIR = "/sim_ws"
PARAMS_FILE = os.path.join(WORKSPACE_DIR, "src/rbsf_mvsim/rbsf_mvsim/nodes/params.yaml")
HISTORY_FILE = os.path.join(WORKSPACE_DIR, "src/rbsf_mvsim/rbsf_mvsim/nodes/tuning_history.yaml")
METRICS_DIR = os.path.join(WORKSPACE_DIR, "src/rbsf_mvsim/rbsf_mvsim/metrics")

def load_params() -> Dict[str, Any]:
    """
    Loads parameters from the params.yaml file.

    Returns:
        Dict[str, Any]: A dictionary containing the loaded parameters.
    """
    with open(PARAMS_FILE, "r") as f:
        return yaml.safe_load(f) or {}

def save_params(params: Dict[str, Any]):
    """
    Saves parameters to the params.yaml file.

    Args:
        params (Dict[str, Any]): The parameters dictionary to save.
    """
    with open(PARAMS_FILE, "w") as f:
        yaml.safe_dump(params, f, sort_keys=False)

def get_nested(data: Dict[str, Any], keys: List[str]) -> Any:
    """
    Retrieves a nested value in a dictionary using a list of keys.

    Args:
        data (Dict[str, Any]): The dictionary to search.
        keys (List[str]): The list of keys forming the path to the value.

    Returns:
        Any: The nested value.
    """
    for k in keys:
         data = data[k]
    return data

def set_nested(data: Dict[str, Any], keys: List[str], val: Any):
    """
    Sets a nested value in a dictionary using a list of keys.

    Args:
        data (Dict[str, Any]): The dictionary to modify.
        keys (List[str]): The list of keys forming the path to the value.
        val (Any): The value to set.
    """
    for k in keys[:-1]:
         data = data[k]
    data[keys[-1]] = val

def list_metric_files() -> List[str]:
    """
    Lists all available generated metric files.

    Returns:
        List[str]: A list of metric file paths.
    """
    return glob.glob(os.path.join(METRICS_DIR, "lap_*.json"))

def objective_function(x: List[float], tunable_keys: List[str], params: Dict[str, Any]) -> float:
    """
    Objective function to MINIMIZE by the Bayesian optimizer.
    Starts the simulation, obtains lap metrics, and translates them into a score.

    Args:
        x (List[float]): Current parameter values chosen by the optimizer.
        tunable_keys (List[str]): List of parameter keys corresponding to x.
        params (Dict[str, Any]): The full parameters dictionary.

    Returns:
        float: The calculated score (cost) for the given parameters.
    """
    for i, keys_str in enumerate(tunable_keys):
        set_nested(params, keys_str.split('.'), float(x[i]))
    
    save_params(params)
    print(f"\n[Optimizer] Evaluating parameters: {dict(zip(tunable_keys, x))}")

    baseline_metrics = set(list_metric_files())

    cmd = (
        "cd /sim_ws && "
        "source install/local_setup.bash && "
        "ros2 launch rbsf_mvsim rbsf_plan_control.launch.py world_file:=src/rbsf_mvsim/rbsf_mvsim/f1tenth_catalunya.world.xml"
    )
    
    print(f"[Optimizer] Starting ROS 2 simulation...")
    
    status_file = '/sim_ws/src/rbsf_mvsim/rbsf_mvsim/metrics/current_status.json'
    if os.path.exists(status_file):
        try:
            os.remove(status_file)
        except Exception:
            pass

    process = subprocess.Popen(
        cmd, 
        shell=True, 
        executable="/bin/bash",
        preexec_fn=os.setsid,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

    score = 1000.0 
    timeout_sec = params.get("auto_tuner", {}).get("workflow", {}).get("lap_timeout_sec", 300.0)
    start_time = time.time()
    
    try:
        while time.time() - start_time < timeout_sec:
            current_metrics = set(list_metric_files())
            new_files = list(current_metrics - baseline_metrics)
            
            if new_files:
                new_files.sort(key=os.path.getmtime, reverse=True)
                latest_metric = new_files[0]
                
                time.sleep(0.5)
                
                try:
                    with open(latest_metric, "r") as f:
                        data = json.load(f)
                    
                    lap_time = data.get("lap_time")
                    lap_metrics = data.get("lap_metrics", [])
                    
                    if lap_time and lap_metrics:
                        avg_cte = sum(m.get("cte", 0.0) for m in lap_metrics) / len(lap_metrics)
                        avg_speed_err = sum(m.get("speed_error", 0.0) for m in lap_metrics) / len(lap_metrics)
                        
                        score = lap_time + (5.0 * avg_cte) + (1.0 * avg_speed_err)
                        print(f"[Optimizer] Lap completed. lap_time: {lap_time:.2f}, avg_cte: {avg_cte:.4f}, current_score: {score:.2f}")
                    else:
                        print("[Optimizer] File exists but metric data is incomplete (possible crash).")
                        
                except Exception as e:
                    print(f"[Optimizer] Error reading JSON metrics: {e}")
                
                break
                
            if process.poll() is not None:
                print(f"[Optimizer] The simulation ended unexpectedly (crash/fatal ROS error).")
                break
                
            time.sleep(1.0)
        else:
            print("[Optimizer] Simulation timed out! Applying progress-based penalty.")
            if os.path.exists(status_file):
                try:
                    with open(status_file, "r") as f:
                        status = json.load(f)
                    progress = status.get("progress", 0.0)
                    avg_cte = status.get("avg_cte", 0.0)
                    avg_speed_err = status.get("avg_speed_error", 0.0)
                    
                    lap_time = 1000.0 - (500.0 * progress)
                    score = lap_time + (5.0 * avg_cte) + (1.0 * avg_speed_err)
                    print(f"[Optimizer] Timeout score calculated: {score:.2f} (Progress: {progress:.2f})")
                except Exception as e:
                    print(f"[Optimizer] Error reading timeout status: {e}. Using default penalty.")
            else:
                print("[Optimizer] No status file found (car likely didn't move). Using default penalty.")
    finally:
        print("[Optimizer] Stopping current simulation...")
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=5.0)
        except Exception:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except Exception:
                pass
        
        time.sleep(3.0)
        
    return score

class TuningLogger:
    """Callback to save tuning history to a YAML file."""
    def __init__(self, filepath: str, tunable_keys: List[str], dimensions: List[Any]):
        self.filepath = filepath
        self.tunable_keys = tunable_keys
        
        # Extract bounds from dimensions for plotting context
        self.bounds = {}
        for dim, key in zip(dimensions, tunable_keys):
            self.bounds[key] = {"low": float(dim.low), "high": float(dim.high)}

    def __call__(self, res):
        history = []
        best_score = float('inf')
        best_params = None
        
        for i, (x_val, y_val) in enumerate(zip(res.x_iters, res.func_vals)):
            current_score = float(y_val)
            current_x = [float(val) for val in x_val]
            
            if current_score < best_score:
                best_score = current_score
                best_params = current_x
                
            entry = {
                "iteration": i + 1,
                "current_params": dict(zip(self.tunable_keys, current_x)),
                "current_score": current_score,
                "best_params": dict(zip(self.tunable_keys, best_params)),
                "best_score": best_score
            }
            history.append(entry)
            
        output_data = {
            "metadata": {
                "tunable_keys": self.tunable_keys,
                "bounds": self.bounds
            },
            "tuning_history": history
        }
        
        with open(self.filepath, "w") as f:
            yaml.safe_dump(output_data, f, sort_keys=False)
            
        new_entry = history[-1]
        print(f"\n[Optimizer] --- Iteration {len(history)} Completed ---")
        print(f"[Optimizer] Score: {new_entry['current_score']:.4f} (Best so far: {new_entry['best_score']:.4f})")

def main():
    """
    Main execution block for Bayesian tuning.
    """
    original_params = load_params()
    
    tuner_cfg = original_params.get("auto_tuner", {})
    tunable_keys = tuner_cfg.get("tunable", [])
    bounds_cfg = tuner_cfg.get("bounds", {})
    iterations = int(tuner_cfg.get("iterations", 20))
    resume = tuner_cfg.get("resume", True)
    
    if not tunable_keys:
        print("No tunable parameters ('tunable') specified in the auto_tuner block inside params.yaml")
        print("Example usage in params.yaml:")
        print("auto_tuner:\n  tunable:\n    - mpc_controller.w_y")
        return

    dimensions = []
    default_x0 = []
    
    for k in tunable_keys:
        val = float(get_nested(original_params, k.split('.')))
        default_x0.append(val)
        
        if k in bounds_cfg:
            low, high = bounds_cfg[k]
        else:
            margin = max(abs(val) * 0.5, 0.1)
            low, high = val - margin, val + margin
            
        dimensions.append(Real(float(low), float(high), name=k))
        
    x0_args = {"x0": [default_x0]}
    
    if resume and os.path.exists(HISTORY_FILE):
        try:
            with open(HISTORY_FILE, "r") as f:
                history_data = yaml.safe_load(f)
            
            hist_keys = history_data.get("metadata", {}).get("tunable_keys", [])
            if hist_keys == tunable_keys:
                print(f"[Optimizer] Resuming from {HISTORY_FILE}...")
                hist_x0 = []
                hist_y0 = []
                for entry in history_data.get("tuning_history", []):
                    params_dict = entry["current_params"]
                    hist_x0.append([params_dict[k] for k in tunable_keys])
                    hist_y0.append(entry["current_score"])
                
                if hist_x0:
                    x0_args = {"x0": hist_x0, "y0": hist_y0}
                    print(f"[Optimizer] Loaded {len(hist_x0)} previous evaluations. Will perform {iterations} more new evaluations.")
            else:
                print("[Optimizer] WARNING: tunable_keys in history do not match current tunable_keys. Starting fresh.")
        except Exception as e:
            print(f"[Optimizer] Error loading history for resume: {e}. Starting fresh.")
    elif not resume and os.path.exists(HISTORY_FILE):
        print(f"[Optimizer] Note: {HISTORY_FILE} exists but 'resume: true' is not set in params.yaml. Overwriting history.")
        
    print("==================================================================")
    print(" Starting Bayesian Optimization (gp_minimize from scikit-optimize)")
    print(f" Total NEW iterations to perform: {iterations}")
    print(f" Parameters to tune: {tunable_keys}")
    print("==================================================================\n")
    
    res = gp_minimize(
        func=lambda x: objective_function(x, tunable_keys, original_params),
        dimensions=dimensions,
        n_calls=iterations,
        random_state=42,
        verbose=False,
        callback=[TuningLogger(HISTORY_FILE, tunable_keys, dimensions)],
        **x0_args
    )
    
    print("\n==================================================================")
    print(" Optimization Completed Successfully!")
    print(f" Best 'score' obtained (Minimized): {res.fun:.4f}")
    print(" With the following parameters:")
    
    for i, key in enumerate(tunable_keys):
        best_val = res.x[i]
        print(f"  {key}: {best_val}")
        set_nested(original_params, key.split('.'), float(best_val))
        
    save_params(original_params)
    print("\n Optimal parameters have been automatically overwritten in params.yaml.")

if __name__ == "__main__":
    main()