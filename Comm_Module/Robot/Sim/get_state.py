#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仿真机器人（IsaacLab EnvTest）状态获取。

从正在运行的 envtest_model_use_player.py 进程和控制文件中读取状态，
映射为通用 observation 结构。
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

try:
    from Comm_Module.Robot.Sim.envtest_status_sync import (
        DEFAULT_CONTROL_FILES,
        DEFAULT_ENVTEST_REPO_ROOT,
        MODEL_USE_TO_SKILL,
        _build_scene_objects,
        _extract_cli_arg,
        _find_envtest_player_process,
        _read_bool_file,
        _read_numeric_file,
        _read_status_json_file,
        _read_vector_file,
        _round_list,
    )
except ImportError:
    from envtest_status_sync import (
        DEFAULT_CONTROL_FILES,
        DEFAULT_ENVTEST_REPO_ROOT,
        MODEL_USE_TO_SKILL,
        _build_scene_objects,
        _extract_cli_arg,
        _find_envtest_player_process,
        _read_bool_file,
        _read_numeric_file,
        _read_status_json_file,
        _read_vector_file,
        _round_list,
    )


def get_state() -> dict[str, Any]:
    """获取仿真机器人的当前状态，映射为通用 observation 结构。

    Returns:
        {
            "connected": bool,
            "robot_type": "sim",
            "observation": {
                "agent_position": list[float] | None,    机器人当前位置 [x, y, z]
                "environment": {
                    "scene_id": int | None,              当前场景
                    "obstacles": list[dict],              场景物体（平台、箱子等）
                    "goal": list[float] | None,           导航目标点
                },
                "action_result": {
                    "skill": str | None,                  当前执行的技能
                    "model_use": int | None,              EnvTest 模型编号
                    "vel_command": list[float] | None,    速度命令
                    "start": bool | None,                 是否已启动
                },
                # Sim-specific raw state
                "raw": {                                  EnvTest 原始状态，供直接访问
                    "pose_command": ...,
                    "status_file_available": ...,
                    ...
                }
            }
        }
    """
    process_info = _find_envtest_player_process()
    if process_info is None:
        return {
            "connected": False,
            "robot_type": "sim",
            "observation": {"agent_position": None, "environment": {}, "action_result": {}, "raw": {}},
        }

    _, tokens = process_info
    repo_root = Path(
        _extract_cli_arg(tokens, "--repo_root", str(DEFAULT_ENVTEST_REPO_ROOT))
        or DEFAULT_ENVTEST_REPO_ROOT
    )
    scene_id_raw = _extract_cli_arg(tokens, "--scene_id", "0") or "0"
    try:
        scene_id = int(scene_id_raw)
    except ValueError:
        scene_id = 0

    model_use_file = Path(
        _extract_cli_arg(tokens, "--model_use_file", str(DEFAULT_CONTROL_FILES["model_use_file"]))
        or DEFAULT_CONTROL_FILES["model_use_file"]
    )
    velocity_file = Path(
        _extract_cli_arg(tokens, "--velocity_command_file", str(DEFAULT_CONTROL_FILES["velocity_file"]))
        or DEFAULT_CONTROL_FILES["velocity_file"]
    )
    goal_file = Path(
        _extract_cli_arg(tokens, "--goal_command_file", str(DEFAULT_CONTROL_FILES["goal_file"]))
        or DEFAULT_CONTROL_FILES["goal_file"]
    )
    status_file = Path(
        _extract_cli_arg(tokens, "--status_json_file", str(DEFAULT_CONTROL_FILES["status_file"]))
        or DEFAULT_CONTROL_FILES["status_file"]
    )
    start_file = Path(
        _extract_cli_arg(tokens, "--start_file", str(DEFAULT_CONTROL_FILES["start_file"]))
        or DEFAULT_CONTROL_FILES["start_file"]
    )

    snapshot: dict[str, Any] = {
        "scene_id": scene_id,
        "model_use": _read_numeric_file(model_use_file),
        "start": _read_bool_file(start_file),
        "vel_command": _read_vector_file(velocity_file, (3,)),
        "pose_command": _read_vector_file(goal_file, (3, 4)),
        "goal": _read_vector_file(goal_file, (3, 4)),
        "robot_pose": [0.0, 0.0, 0.0],
    }

    model_use = snapshot.get("model_use")
    if isinstance(model_use, int):
        snapshot["skill"] = MODEL_USE_TO_SKILL.get(model_use)

    status_payload = _read_status_json_file(status_file)
    status_file_available = status_payload is not None
    if status_payload:
        for key in ("model_use", "skill", "scene_id", "start", "unified_obs_dim", "policy_obs_dim"):
            if key in status_payload:
                snapshot[key] = status_payload.get(key)
        for key in ("pose_command", "vel_command", "robot_pose", "goal"):
            values = status_payload.get(key)
            if isinstance(values, list):
                snapshot[key] = _round_list(values)

    scene_objects: list[dict[str, Any]] = []
    try:
        scene_objects = _build_scene_objects(scene_id, repo_root)
    except Exception:
        scene_objects = []

    return {
        "connected": True,
        "robot_type": "sim",
        "observation": {
            "agent_position": snapshot.get("robot_pose"),
            "environment": {
                "scene_id": snapshot.get("scene_id"),
                "obstacles": scene_objects,
                "goal": snapshot.get("goal"),
            },
            "action_result": {
                "skill": snapshot.get("skill"),
                "model_use": snapshot.get("model_use"),
                "vel_command": snapshot.get("vel_command"),
                "start": snapshot.get("start"),
            },
            "raw": {
                "pose_command": snapshot.get("pose_command"),
                "status_file_available": status_file_available,
            },
        },
    }


def main() -> int:
    state = get_state()
    print(json.dumps(state, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
