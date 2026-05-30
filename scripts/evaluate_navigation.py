#!/usr/bin/env python3
"""Run automatic navigation evaluation over IsaacLab scenes.

The script launches the IsaacLab stack through a temporary scene-specific copy
of run_isaaclab.sh, sends the fixed user instruction to the planner, executes
the returned tool queue, and marks a trial as passed when the final XY error is
within the threshold.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import tempfile
import time
from datetime import datetime
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from Executor.executor import run_plan
from Executor.robot_ws import check_connection, get_robot_state
from Executor.state import format_feedback, format_latest_state
from Planner.llm_core import make_plan, prompt
from Tui.planning import has_action_tool, incomplete_tool_batch, planning_messages, should_replan_after_batch


DEFAULT_ISAACLAB_SCRIPT = Path("/home/xcj/work/IsaacLab/IsaacLabBisShe/run_isaaclab.sh")
DEFAULT_REPORT_DIR = PROJECT_ROOT / "eval_results"
ACTION_TOOLS = {"nav", "walk_skill", "push", "climb"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate navigation task success over scenes 0-4.")
    parser.add_argument("--isaaclab-script", type=Path, default=DEFAULT_ISAACLAB_SCRIPT)
    parser.add_argument("--scenes", type=int, nargs="+", default=[0, 1, 2, 3, 4])
    parser.add_argument("--trials", type=int, default=25)
    parser.add_argument("--instruction", default="导航到目标点")
    parser.add_argument("--threshold", type=float, default=0.25)
    parser.add_argument("--startup-timeout", type=float, default=180.0)
    parser.add_argument("--trial-timeout", type=float, default=240.0)
    parser.add_argument("--planner-rounds", type=int, default=10)
    parser.add_argument("--reset-between-trials", action="store_true", default=True)
    parser.add_argument("--restart-each-trial", action="store_true", help="Restart IsaacLab for every trial instead of once per scene.")
    parser.add_argument("--target-json", type=Path, help="Optional scene target map, e.g. {\"0\":[4,0,0.3]}.")
    parser.add_argument("--report-dir", type=Path, default=DEFAULT_REPORT_DIR)
    parser.add_argument("--keep-sim-on-failure", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.report_dir.mkdir(parents=True, exist_ok=True)
    expected_targets = load_expected_targets(args.target_json)
    started_at = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_path = args.report_dir / f"navigation_eval_{started_at}.json"
    summary_path = args.report_dir / f"navigation_eval_{started_at}.md"

    report = {
        "instruction": args.instruction,
        "threshold_m": args.threshold,
        "scenes": args.scenes,
        "trials_per_scene": args.trials,
        "started_at": started_at,
        "results": [],
    }

    for scene_id in args.scenes:
        scene_process = None
        scene_log = None
        try:
            if not args.restart_each_trial:
                scene_process, scene_log = start_isaaclab(args, scene_id, trial_label="scene")
                wait_robot_service(args.startup_timeout)
            for trial_index in range(args.trials):
                if args.restart_each_trial:
                    scene_process, scene_log = start_isaaclab(args, scene_id, trial_label=f"trial{trial_index + 1:02d}")
                    wait_robot_service(args.startup_timeout)
                elif args.reset_between_trials:
                    reset_sim_state()
                    wait_robot_service(30.0)

                trial = run_trial(args, scene_id, trial_index + 1, expected_targets.get(str(scene_id)))
                trial["scene_log"] = str(scene_log) if scene_log else None
                report["results"].append(trial)
                write_reports(report, report_path, summary_path)
                print_trial(trial)

                if args.restart_each_trial:
                    stop_process(scene_process)
                    scene_process = None
                    scene_log = None
        finally:
            if scene_process is not None and (not args.keep_sim_on_failure or all_ok(report["results"], scene_id)):
                stop_process(scene_process)

    write_reports(report, report_path, summary_path)
    passed = sum(1 for item in report["results"] if item.get("passed"))
    total = len(report["results"])
    print(f"\n[eval] finished: {passed}/{total} passed")
    print(f"[eval] json report: {report_path}")
    print(f"[eval] markdown report: {summary_path}")
    return 0 if passed == total else 1


def start_isaaclab(args: argparse.Namespace, scene_id: int, trial_label: str):
    launch_script = make_scene_launch_script(args.isaaclab_script, scene_id)
    log_path = args.report_dir / f"isaaclab_scene{scene_id}_{trial_label}_{datetime.now().strftime('%H%M%S')}.log"
    log_file = log_path.open("w", encoding="utf-8")
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    process = subprocess.Popen(
        ["bash", str(launch_script)],
        cwd=str(args.isaaclab_script.parent),
        stdout=log_file,
        stderr=subprocess.STDOUT,
        env=env,
        preexec_fn=os.setsid,
        text=True,
    )
    print(f"[eval] launched scene={scene_id}, pid={process.pid}, log={log_path}")
    return process, log_path


def make_scene_launch_script(base_script: Path, scene_id: int) -> Path:
    text = base_script.read_text(encoding="utf-8")
    if "--scene_id" not in text:
        raise RuntimeError(f"{base_script} does not contain --scene_id")
    text = re.sub(r"--scene_id\s+\d+", f"--scene_id {scene_id}", text)
    temp_path = Path(tempfile.gettempdir()) / f"run_isaaclab_scene_{scene_id}.sh"
    temp_path.write_text(text, encoding="utf-8")
    temp_path.chmod(0o755)
    return temp_path


def wait_robot_service(timeout_sec: float) -> None:
    deadline = time.time() + timeout_sec
    last = None
    while time.time() < deadline:
        result = check_connection(timeout_sec=3)
        last = result
        if result.get("signal") == "SUCCESS" and result.get("status_json_ready"):
            print("[eval] robot service ready")
            return
        time.sleep(2.0)
    raise TimeoutError(f"robot service not ready after {timeout_sec}s: {last}")


def reset_sim_state() -> None:
    write_text("/tmp/model_use.txt", "0")
    write_text("/tmp/envtest_start.txt", "0")
    write_text("/tmp/envtest_velocity_command.txt", "0 0 0")
    write_text("/tmp/envtest_reset.txt", "1")
    time.sleep(2.0)


def run_trial(args: argparse.Namespace, scene_id: int, trial_no: int, expected_target):
    print(f"[eval] scene={scene_id} trial={trial_no}/{args.trials}: {args.instruction}")
    started = time.time()
    messages = [
        {"role": "system", "content": prompt["system_prompt"]},
        {"role": "user", "content": args.instruction},
    ]
    observed = False
    executed_calls = []
    events = []
    planning_duration = 0.0
    planner_calls = 0
    final_error = None
    final_state = None
    final_target = None
    failure = None

    def emit(event_type, content):
        events.append({"type": event_type, "content": str(content)})
        if event_type in {"tool", "error"}:
            print(f"[{event_type}] {content}")

    try:
        for _round in range(args.planner_rounds):
            if time.time() - started > args.trial_timeout:
                failure = f"trial timeout after {args.trial_timeout}s"
                break
            planner_started = time.time()
            try:
                result = make_plan(planning_messages(messages, observed))
            finally:
                planning_duration += time.time() - planner_started
                planner_calls += 1
            if result["type"] != "plan":
                failure = f"planner returned text instead of plan: {result.get('content')}"
                break

            tool_calls = result["tool_calls"]
            if has_action_tool(tool_calls) and not observed:
                tool_calls = [{"name": "observe", "args": {}}]
            elif incomplete_tool_batch(result.get("content"), tool_calls):
                messages.append({"role": "assistant", "content": result.get("content") or ""})
                messages.append({
                    "role": "user",
                    "content": "你刚才列出了多步动作计划，但没有把每一步都放进tool_calls。请一次性返回完整动作队列的tool_calls序列。",
                })
                continue

            plan_results = run_plan(tool_calls, emit)
            executed_calls.extend(tool_calls)
            if any(item["name"] == "observe" and item["signal"] != "FAILURE" for item in plan_results):
                observed = True

            tool_result = summarize_plan_results(plan_results, format_latest_state(), format_feedback())
            messages.append({"role": "assistant", "content": f"已执行: {format_steps(tool_calls)}\n结果: {tool_result}"})
            if not should_replan_after_batch(plan_results, tool_calls):
                break
            messages.append({"role": "user", "content": f"执行结果: {tool_result}，请决定下一步"})

        final_state = safe_get_robot_state()
        final_target = expected_target or infer_final_target(executed_calls)
        final_error = compute_error(final_state, final_target)
    except Exception as error:
        failure = str(error)
        final_state = safe_get_robot_state()
        final_target = expected_target or infer_final_target(executed_calls)
        final_error = compute_error(final_state, final_target)

    passed = final_error is not None and final_error <= args.threshold
    duration = time.time() - started
    return {
        "scene_id": scene_id,
        "trial": trial_no,
        "instruction": args.instruction,
        "passed": passed,
        "error_xy_m": final_error,
        "threshold_m": args.threshold,
        "target": final_target,
        "final_state": final_state,
        "executed_calls": executed_calls,
        "failure": failure,
        "duration_sec": round(duration, 3),
        "planning_duration_sec": round(planning_duration, 3),
        "execution_duration_sec": round(max(0.0, duration - planning_duration), 3),
        "planner_calls": planner_calls,
        "events_tail": events[-20:],
    }


def summarize_plan_results(plan_results, latest_state, latest_feedback):
    return json.dumps({
        "steps": plan_results or [],
        "latest_state": latest_state,
        "latest_feedback": latest_feedback,
    }, ensure_ascii=False)


def format_steps(tool_calls):
    return ", ".join(f"{tc['name']}({tc['args']})" for tc in tool_calls)


def infer_final_target(executed_calls):
    for call in reversed(executed_calls):
        name = call.get("name")
        args = dict(call.get("args") or {})
        if name == "nav" and {"x", "y"}.issubset(args):
            return {"kind": "robot", "x": float(args["x"]), "y": float(args["y"]), "z": float(args.get("z", 0.0))}
        if name == "push" and {"x", "y"}.issubset(args):
            return {"kind": "box_world", "x": float(args["x"]), "y": float(args["y"]), "z": float(args.get("z", 0.0))}
    return None


def compute_error(final_state, target):
    if not final_state or not target:
        return None
    key = target.get("kind", "robot")
    position = final_state.get(key) or final_state.get("robot")
    if not isinstance(position, dict):
        return None
    return round(math.hypot(float(position.get("x", 0.0)) - float(target["x"]), float(position.get("y", 0.0)) - float(target["y"])), 4)


def safe_get_robot_state():
    try:
        state = get_robot_state(timeout_sec=3)
        state.pop("_revision", None)
        return state
    except Exception as error:
        return {"signal": "FAILURE", "message": str(error)}


def load_expected_targets(path):
    if not path:
        return {}
    data = json.loads(path.read_text(encoding="utf-8"))
    targets = {}
    for scene, value in data.items():
        if isinstance(value, dict):
            targets[str(scene)] = value
        else:
            xyz = list(value) + [0.0, 0.0, 0.0]
            targets[str(scene)] = {"kind": "robot", "x": float(xyz[0]), "y": float(xyz[1]), "z": float(xyz[2])}
    return targets


def write_reports(report, json_path: Path, md_path: Path) -> None:
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    rows = ["# Navigation Evaluation", "", f"- instruction: `{report['instruction']}`", f"- threshold: `{report['threshold_m']} m`", ""]
    rows.append("| scene | trial | pass | error_xy_m | target | planning_s | duration_s | failure |")
    rows.append("|---:|---:|:---:|---:|---|---:|---:|---|")
    for item in report["results"]:
        rows.append(
            f"| {item['scene_id']} | {item['trial']} | {'PASS' if item['passed'] else 'FAIL'} "
            f"| {item.get('error_xy_m')} | `{item.get('target')}` | {item.get('planning_duration_sec')} "
            f"| {item.get('duration_sec')} | {item.get('failure') or ''} |"
        )
    md_path.write_text("\n".join(rows) + "\n", encoding="utf-8")


def print_trial(trial):
    status = "PASS" if trial.get("passed") else "FAIL"
    print(
        f"[eval] scene={trial['scene_id']} trial={trial['trial']} {status} "
        f"error={trial.get('error_xy_m')} target={trial.get('target')} duration={trial.get('duration_sec')}s"
    )


def stop_process(process):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        process.wait(timeout=10)
    except Exception:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        except Exception:
            pass


def all_ok(results, scene_id):
    return all(item.get("passed") for item in results if item.get("scene_id") == scene_id)


def write_text(path, text):
    Path(path).write_text(str(text).strip() + "\n", encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
