import argparse
import copy
import glob
import json
import os
import random
import shlex
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml


@dataclass
class EvaluationResult:
    """
    Data class representing the result of a single parameter evaluation.
    
    Attributes:
        score (float): The final score (objective value).
        lap_completed (bool): Whether the lap was fully completed.
        lap_time (Optional[float]): Total time taken to complete the lap.
        avg_cte (Optional[float]): Average cross-track error during the lap.
        avg_speed_error (Optional[float]): Average speed error during the lap.
        metrics_file (Optional[str]): Path to the metrics JSON file.
        reason (str): Description of the evaluation outcome or failure reason.
    """
    score: float
    lap_completed: bool
    lap_time: Optional[float]
    avg_cte: Optional[float]
    avg_speed_error: Optional[float]
    metrics_file: Optional[str]
    reason: str


class AutoParamTuner:
    """
    Automatic parameter tuner using simulation metrics to optimize parameters.
    
    Attributes:
        params_path (Path): Path to the params.yaml file.
        random (random.Random): Random number generator for proposing candidates.
        params (Dict): Dictionary of current parameters.
        tuner_cfg (Dict): Tuner configuration from params.yaml.
        workflow_cfg (Dict): Workflow configuration from params.yaml.
    """
    def __init__(self, params_path: str, seed: int = 42):
        """
        Initializes the auto tuner.

        Args:
            params_path (str): The path to the params.yaml file.
            seed (int, optional): Random seed for reproducibility. Defaults to 42.
        
        Raises:
            FileNotFoundError: If the params.yaml file is not found.
            ValueError: If there are no tunable parameters or configuration is invalid.
        """
        self.params_path = Path(params_path)
        if not self.params_path.exists():
            raise FileNotFoundError(f"params.yaml does not exist at: {self.params_path}")

        self.random = random.Random(seed)
        self._load_params()

        self.tuner_cfg = self.params.get("auto_tuner", {})
        self.workflow_cfg = self.tuner_cfg.get("workflow", {})

        self.iterations = int(self.tuner_cfg.get("iterations", 20))
        self.improvement_epsilon = float(self.tuner_cfg.get("improvement_epsilon", 1e-6))
        self.rebuild_every_iteration = bool(self.workflow_cfg.get("rebuild_every_iteration", True))
        self.verbose = bool(self.tuner_cfg.get("verbose", True))

        # Objective weights
        objective = self.tuner_cfg.get("objective", {})
        self.weight_lap_time = float(objective.get("lap_time_weight", 1.0))
        self.weight_cte = float(objective.get("cte_weight", 5.0))
        self.weight_speed_error = float(objective.get("speed_error_weight", 1.0))
        self.crash_penalty = float(objective.get("crash_penalty", 1e6))

        # Workflow commands
        self.source_cmd = self.workflow_cfg.get("source_cmd", "")
        self.build_cmd = self.workflow_cfg.get("build_cmd", "")
        self.simulation_cmd = self.workflow_cfg.get("simulation_cmd", "")
        self.node_start_order = list(self.workflow_cfg.get("node_start_order", []))
        self.stop_cmd = self.workflow_cfg.get("stop_cmd", "")
        self.use_interactive_shell = bool(self.workflow_cfg.get("use_interactive_shell", False))
        self.simulation_use_interactive_shell = bool(
            self.workflow_cfg.get("simulation_use_interactive_shell", "RUN_SIM" in self.simulation_cmd)
        )
        self.alias_setup_cmd = self.workflow_cfg.get("alias_setup_cmd", "")

        self.metrics_glob = str(self.workflow_cfg.get("metrics_glob", ""))
        self.lap_timeout_sec = float(self.workflow_cfg.get("lap_timeout_sec", 90.0))
        self.settle_time_sec = float(self.workflow_cfg.get("settle_time_sec", 5.0))
        self.node_start_gap_sec = float(self.workflow_cfg.get("node_start_gap_sec", 1.0))
        self.shutdown_grace_sec = float(self.workflow_cfg.get("shutdown_grace_sec", 8.0))
        self.build_timeout_sec = float(self.workflow_cfg.get("build_timeout_sec", 1200.0))
        self.metrics_read_timeout_sec = float(self.workflow_cfg.get("metrics_read_timeout_sec", 10.0))
        self.metrics_stable_wait_sec = float(self.workflow_cfg.get("metrics_stable_wait_sec", 0.4))

        if not self.metrics_glob:
            default_metrics = self.params_path.parent.parent / "metrics" / "lap_*.json"
            self.metrics_glob = str(default_metrics)

        self.result_path = self.params_path.parent / "tuning_results.json"

        self.tunable_keys = self._resolve_tunable_keys()
        if not self.tunable_keys:
            raise ValueError(
                "No tunable parameters. Check auto_tuner.tunable or the filter in auto_tuner.locked."
            )

        self.bounds = self._resolve_bounds(self.tunable_keys)
        self.steps = self._resolve_steps(self.tunable_keys)

        self._validate_workflow_config()

    def _load_params(self) -> None:
        with self.params_path.open("r", encoding="utf-8") as f:
            self.params = yaml.safe_load(f) or {}

    def _save_params(self) -> None:
        with self.params_path.open("w", encoding="utf-8") as f:
            yaml.safe_dump(self.params, f, sort_keys=False, allow_unicode=False)

    def _validate_workflow_config(self) -> None:
        if not self.simulation_cmd:
            raise ValueError("Missing auto_tuner.workflow.simulation_cmd in params.yaml")
        if not self.node_start_order:
            raise ValueError("Missing auto_tuner.workflow.node_start_order in params.yaml")
        if not self.build_cmd and self.rebuild_every_iteration:
            raise ValueError("Missing auto_tuner.workflow.build_cmd with rebuild_every_iteration=true")

    def _log(self, message: str) -> None:
        if self.verbose:
            print(f"[TUNER] {message}", flush=True)

    def _flatten_numeric_leaf_keys(self, data: Dict[str, Any], prefix: str = "") -> List[str]:
        keys: List[str] = []
        for k, v in data.items():
            if k == "auto_tuner":
                continue
            path = f"{prefix}.{k}" if prefix else k
            if isinstance(v, dict):
                keys.extend(self._flatten_numeric_leaf_keys(v, path))
            elif isinstance(v, (int, float)) and not isinstance(v, bool):
                keys.append(path)
        return keys

    def _resolve_tunable_keys(self) -> List[str]:
        all_numeric = self._flatten_numeric_leaf_keys(self.params)

        locked = set(self.tuner_cfg.get("locked", []))
        explicit_tunable = self.tuner_cfg.get("tunable", [])

        if explicit_tunable:
            tunable = [k for k in explicit_tunable if k in all_numeric and k not in locked]
        else:
            tunable = [k for k in all_numeric if k not in locked]

        return tunable

    def _resolve_bounds(self, keys: List[str]) -> Dict[str, Tuple[float, float]]:
        configured = self.tuner_cfg.get("bounds", {})
        out: Dict[str, Tuple[float, float]] = {}

        for k in keys:
            current = float(self._get_nested(k))
            if k in configured:
                b = configured[k]
                if not isinstance(b, list) or len(b) != 2:
                    raise ValueError(f"Invalid bounds for {k}. Use [min, max].")
                low, high = float(b[0]), float(b[1])
            else:
                span = max(abs(current) * 0.5, 1e-3)
                low, high = current - span, current + span
            if low > high:
                low, high = high, low
            out[k] = (low, high)

        return out

    def _resolve_steps(self, keys: List[str]) -> Dict[str, float]:
        configured = self.tuner_cfg.get("steps", {})
        steps: Dict[str, float] = {}

        for k in keys:
            current = float(self._get_nested(k))
            if k in configured:
                step = float(configured[k])
            else:
                step = max(abs(current) * 0.1, 1e-3)
            if step <= 0.0:
                raise ValueError(f"Invalid step for {k}: {step}")
            steps[k] = step

        return steps

    def _get_nested(self, dotted_key: str) -> Any:
        ref: Any = self.params
        for part in dotted_key.split("."):
            ref = ref[part]
        return ref

    def _set_nested(self, dotted_key: str, value: Any) -> None:
        parts = dotted_key.split(".")
        ref: Any = self.params
        for part in parts[:-1]:
            ref = ref[part]

        existing = ref[parts[-1]]
        if isinstance(existing, int) and not isinstance(existing, bool):
            ref[parts[-1]] = int(round(value))
        else:
            ref[parts[-1]] = float(value)

    def _clamp(self, key: str, value: float) -> float:
        low, high = self.bounds[key]
        return max(low, min(high, value))

    def _run_shell(self, command: str, timeout: Optional[float] = None, check: bool = True) -> subprocess.CompletedProcess:
        parts: List[str] = []
        if self.alias_setup_cmd:
            parts.append("shopt -s expand_aliases")
            parts.append(self.alias_setup_cmd)
        if self.source_cmd:
            parts.append(f"source {self.source_cmd}")
        parts.append(command)
        full_cmd = " && ".join(parts)

        return subprocess.run(
            ["bash", "-lc", full_cmd],
            check=check,
            timeout=timeout,
            capture_output=True,
            text=True,
        )

    def _start_process(self, command: str, interactive_shell: bool = False) -> subprocess.Popen:
        parts: List[str] = []
        if self.alias_setup_cmd:
            parts.append("shopt -s expand_aliases")
            parts.append(self.alias_setup_cmd)
        if self.source_cmd:
            parts.append(f"source {self.source_cmd}")
        parts.append(command)
        full_cmd = " && ".join(parts)

        if self.use_interactive_shell and not interactive_shell:
            self._log(
                "Warning: use_interactive_shell=true detected, but forcing bash -lc to avoid job control suspension."
            )

        bash_mode = "-ic" if interactive_shell else "-lc"

        process = subprocess.Popen(
            ["bash", bash_mode, full_cmd],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid,
        )
        return process

    def _check_command_available(self, command_name: str, interactive_shell: bool = False) -> bool:
        probe = f"type {shlex.quote(command_name)} >/dev/null 2>&1"
        parts: List[str] = []
        if self.alias_setup_cmd:
            parts.append("shopt -s expand_aliases")
            parts.append(self.alias_setup_cmd)
        if self.source_cmd:
            parts.append(f"source {self.source_cmd}")
        parts.append(probe)
        full_cmd = " && ".join(parts)
        bash_mode = "-ic" if interactive_shell else "-lc"
        proc = subprocess.run(["bash", bash_mode, full_cmd], capture_output=True, text=True)
        return proc.returncode == 0

    def _describe_processes(self, stack_processes: List[subprocess.Popen]) -> str:
        labels = ["simulation"] + [f"node_{i}" for i in range(1, len(stack_processes))]
        states: List[str] = []
        for idx, process in enumerate(stack_processes):
            status = process.poll()
            label = labels[idx] if idx < len(labels) else f"proc_{idx}"
            if status is None:
                states.append(f"{label}:running")
            else:
                states.append(f"{label}:exit({status})")
        return ", ".join(states)

    def _stop_process_with_sigint(self, process: subprocess.Popen) -> None:
        if process.poll() is not None:
            return

        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=self.shutdown_grace_sec)
            return
        except Exception:
            pass

        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=5)
            return
        except Exception:
            pass

        try:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        except Exception:
            pass

    def _stop_stack(self, stack_processes: List[subprocess.Popen]) -> None:
        if self.stop_cmd:
            try:
                self._run_shell(self.stop_cmd, timeout=20, check=False)
            except Exception:
                pass

        for process in reversed(stack_processes):
            self._stop_process_with_sigint(process)

    def _start_stack(self) -> List[subprocess.Popen]:
        stack: List[subprocess.Popen] = []

        if "RUN_SIM" in self.simulation_cmd and not self._check_command_available(
            "RUN_SIM", interactive_shell=self.simulation_use_interactive_shell
        ):
            raise RuntimeError(
                "RUN_SIM does not exist in the execution shell. Define auto_tuner.workflow.alias_setup_cmd "
                "or use simulation_cmd with the real command instead of the alias."
            )

        self._log(f"Starting simulation: {self.simulation_cmd}")
        sim = self._start_process(
            self.simulation_cmd,
            interactive_shell=self.simulation_use_interactive_shell,
        )
        stack.append(sim)

        time.sleep(self.settle_time_sec)
        if sim.poll() is not None:
            raise RuntimeError(f"Simulation terminated after start with code {sim.poll()}")

        for cmd in self.node_start_order:
            self._log(f"Starting node: {cmd}")
            p = self._start_process(cmd, interactive_shell=False)
            stack.append(p)
            time.sleep(self.node_start_gap_sec)
            if p.poll() is not None:
                raise RuntimeError(f"A node terminated right after starting ({cmd}) with code {p.poll()}")

        return stack

    def _list_metric_files(self) -> List[Path]:
        return [Path(p) for p in glob.glob(self.metrics_glob)]

    def _wait_for_new_metric_file(
        self,
        baseline: set,
        timeout_sec: float,
        stack_processes: List[subprocess.Popen],
    ) -> Tuple[Optional[Path], str]:
        end_time = time.time() + timeout_sec
        while time.time() < end_time:
            exited = [p.poll() for p in stack_processes]
            if any(code is not None for code in exited):
                return None, (
                    "at least one process from the stack terminated before completing a lap: "
                    f"{self._describe_processes(stack_processes)}"
                )

            current = self._list_metric_files()
            new_files = [p for p in current if str(p) not in baseline]
            if new_files:
                new_files.sort(key=lambda p: p.stat().st_mtime, reverse=True)
                return new_files[0], "ok"
            time.sleep(0.5)
        return None, "timeout without a complete lap"

    def _read_metric_score(self, metric_file: Path) -> EvaluationResult:
        data = self._read_json_when_ready(metric_file)
        if data is None:
            return EvaluationResult(
                score=self.crash_penalty,
                lap_completed=False,
                lap_time=None,
                avg_cte=None,
                avg_speed_error=None,
                metrics_file=str(metric_file),
                reason="could not read metrics json (partial or invalid file)",
            )

        lap_time = data.get("lap_time")
        lap_metrics = data.get("lap_metrics", [])

        if lap_time is None or not lap_metrics:
            return EvaluationResult(
                score=self.crash_penalty,
                lap_completed=False,
                lap_time=None,
                avg_cte=None,
                avg_speed_error=None,
                metrics_file=str(metric_file),
                reason="incomplete metrics",
            )

        ctes = [float(m.get("cte", 0.0)) for m in lap_metrics]
        speed_errors = [float(m.get("speed_error", 0.0)) for m in lap_metrics]

        avg_cte = sum(ctes) / max(len(ctes), 1)
        avg_speed_error = sum(speed_errors) / max(len(speed_errors), 1)

        score = (
            self.weight_lap_time * float(lap_time)
            + self.weight_cte * avg_cte
            + self.weight_speed_error * avg_speed_error
        )

        return EvaluationResult(
            score=score,
            lap_completed=True,
            lap_time=float(lap_time),
            avg_cte=avg_cte,
            avg_speed_error=avg_speed_error,
            metrics_file=str(metric_file),
            reason="ok",
        )

    def _serialize_result(self, result: EvaluationResult) -> Dict[str, Any]:
        payload: Dict[str, Any] = {
            "objective_score": result.score,
            "score": result.score,
            "lap_completed": result.lap_completed,
            "lap_time": result.lap_time,
            "avg_cte": result.avg_cte,
            "avg_speed_error": result.avg_speed_error,
            "metrics_file": result.metrics_file,
            "lap_time_source": "metrics_file.lap_time",
            "reason": result.reason,
            "objective_breakdown": None,
        }

        if (
            result.lap_time is not None
            and result.avg_cte is not None
            and result.avg_speed_error is not None
        ):
            payload["objective_breakdown"] = {
                "lap_time_term": self.weight_lap_time * result.lap_time,
                "cte_term": self.weight_cte * result.avg_cte,
                "speed_error_term": self.weight_speed_error * result.avg_speed_error,
            }

        return payload

    def _read_json_when_ready(self, json_path: Path) -> Optional[Dict[str, Any]]:
        end_time = time.time() + self.metrics_read_timeout_sec
        last_size = -1

        while time.time() < end_time:
            try:
                size_now = json_path.stat().st_size
            except FileNotFoundError:
                time.sleep(0.1)
                continue

            if size_now <= 0:
                time.sleep(0.1)
                continue

            if size_now == last_size:
                time.sleep(self.metrics_stable_wait_sec)
                try:
                    if json_path.stat().st_size == size_now:
                        with json_path.open("r", encoding="utf-8") as f:
                            return json.load(f)
                except (json.JSONDecodeError, OSError):
                    time.sleep(0.1)
            else:
                last_size = size_now
                time.sleep(0.1)

        return None

    def _evaluate_current_params(self) -> EvaluationResult:
        if self.rebuild_every_iteration:
            try:
                self._log("Build started...")
                self._run_shell(self.build_cmd, timeout=self.build_timeout_sec, check=True)
                self._log("Build completed.")
            except subprocess.TimeoutExpired:
                return EvaluationResult(
                    score=self.crash_penalty,
                    lap_completed=False,
                    lap_time=None,
                    avg_cte=None,
                    avg_speed_error=None,
                    metrics_file=None,
                    reason="build timeout",
                )
            except subprocess.CalledProcessError as e:
                tail = ""
                if e.stdout:
                    tail = e.stdout[-300:]
                return EvaluationResult(
                    score=self.crash_penalty,
                    lap_completed=False,
                    lap_time=None,
                    avg_cte=None,
                    avg_speed_error=None,
                    metrics_file=None,
                    reason=f"build failed: {e.returncode}. tail={tail}",
                )

        baseline_files = {str(p) for p in self._list_metric_files()}

        self._log("Stack start initiated...")
        try:
            stack = self._start_stack()
        except Exception as e:
            return EvaluationResult(
                score=self.crash_penalty,
                lap_completed=False,
                lap_time=None,
                avg_cte=None,
                avg_speed_error=None,
                metrics_file=None,
                reason=f"failed to start stack: {e}",
            )

        self._log("Stack started. Waiting for complete lap...")
        try:
            metric_file, wait_reason = self._wait_for_new_metric_file(
                baseline_files,
                self.lap_timeout_sec,
                stack,
            )
            if metric_file is None:
                return EvaluationResult(
                    score=self.crash_penalty,
                    lap_completed=False,
                    lap_time=None,
                    avg_cte=None,
                    avg_speed_error=None,
                    metrics_file=None,
                    reason=wait_reason,
                )
            self._log(f"Lap detected at: {metric_file}")
            return self._read_metric_score(metric_file)
        finally:
            self._log("Stopping stack...")
            self._stop_stack(stack)
            self._log("Stack stopped.")

    def _capture_current_values(self) -> Dict[str, float]:
        out: Dict[str, float] = {}
        for k in self.tunable_keys:
            out[k] = float(self._get_nested(k))
        return out

    def _apply_values(self, values: Dict[str, float]) -> None:
        for k, v in values.items():
            self._set_nested(k, self._clamp(k, v))
        self._save_params()

    def _propose_candidate(self, best_values: Dict[str, float]) -> Dict[str, float]:
        candidate = copy.deepcopy(best_values)
        for k in self.tunable_keys:
            step = self.steps[k]
            direction = self.random.choice([-1.0, 1.0])
            scale = self.random.uniform(0.5, 1.5)
            trial = candidate[k] + direction * step * scale
            candidate[k] = self._clamp(k, trial)
        return candidate

    def tune(self) -> Dict[str, Any]:
        """
        Runs the tuning loop.

        Returns:
            Dict[str, Any]: Summary of the optimization process including the best parameters.
        """
        baseline_values = self._capture_current_values()

        print("[TUNER] Evaluating baseline...")
        baseline_result = self._evaluate_current_params()
        best_values = baseline_values
        best_result = baseline_result

        history: List[Dict[str, Any]] = []
        history.append(
            {
                "iteration": 0,
                "params": baseline_values,
                "result": self._serialize_result(baseline_result),
            }
        )

        baseline_lap_text = (
            f"{baseline_result.lap_time:.6f}" if baseline_result.lap_time is not None else "None"
        )

        print(
            f"[TUNER] Baseline objective={baseline_result.score:.6f} lap_time={baseline_lap_text} "
            f"lap_completed={baseline_result.lap_completed} reason={baseline_result.reason}"
        )

        for i in range(1, self.iterations + 1):
            candidate = self._propose_candidate(best_values)
            self._apply_values(candidate)

            print(f"[TUNER] Iteration {i}/{self.iterations} evaluating candidate...")
            result = self._evaluate_current_params()

            improved = result.score < (best_result.score - self.improvement_epsilon)
            if improved:
                best_values = candidate
                best_result = result
                for k in self.tunable_keys:
                    self.steps[k] *= 1.1
                status = "IMPROVED"
            else:
                for k in self.tunable_keys:
                    self.steps[k] = max(self.steps[k] * 0.7, 1e-6)
                status = "no improvement"

            history.append(
                {
                    "iteration": i,
                    "params": candidate,
                    "result": self._serialize_result(result),
                    "status": status,
                }
            )

            print(
                f"[TUNER] Iter {i}: score={result.score:.6f} best_result_score={best_result.score:.6f} lap={result.lap_completed} "
                f"status={status} reason={result.reason}"
            )

        self._apply_values(best_values)

        summary = {
            "params_file": str(self.params_path),
            "iterations": self.iterations,
            "tunable_keys": self.tunable_keys,
            "bounds": {k: list(v) for k, v in self.bounds.items()},
            "best_params": best_values,
            "best_result": self._serialize_result(best_result),
            "history": history,
            "timestamp": int(time.time()),
        }

        with self.result_path.open("w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2)

        return summary


def main() -> None:
    """
    Main entry point for the automatic parameter tuner.
    """
    parser = argparse.ArgumentParser(
        description="Automatic parameter tuning of params.yaml using simulation metrics."
    )
    parser.add_argument(
        "--params",
        default="/sim_ws/src/ruben_f1tenth/ruben_f1tenth/nodes/params.yaml",
        help="Path to params.yaml",
    )
    parser.add_argument("--seed", type=int, default=42, help="Randomness seed")
    args = parser.parse_args()

    tuner = AutoParamTuner(params_path=args.params, seed=args.seed)
    summary = tuner.tune()

    best = summary["best_result"]
    print("[TUNER] Optimization finished.")
    print(f"[TUNER] Best objective: {best['objective_score']:.6f}")
    print(f"[TUNER] Best lap_time (metrics): {best['lap_time']}")
    print(f"[TUNER] Best result saved to: {Path(args.params).parent / 'tuning_results.json'}")


if __name__ == "__main__":
    main()
