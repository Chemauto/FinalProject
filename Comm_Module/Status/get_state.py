#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通用状态获取接口。

为 LLM agent 提供统一的状态查询入口，不绑定任何具体机器人或环境。
每个机器人实现只需要返回一个 observation dict，描述 agent 当前能感知到的一切。

架构:
    Status/get_state.py          ← 通用抽象层（本文件）
    Robot/Sim/get_state.py       ← IsaacLab EnvTest 仿真实现
    Robot/Go2/get_state.py       ← 真机实现（未来）
    Robot/Mario/get_state.py     ← 超级玛丽实现（示例）

用法:
    from Comm_Module.Status.get_state import get_state
    state = get_state()           # 自动检测 robot_type
    obs = state["observation"]    # agent 的观测

命令行:
    python -m Comm_Module.Status.get_state
    python -m Comm_Module.Status.get_state --type sim --json
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any

_ROBOT_BACKENDS: dict[str, str] = {
    "sim": "Comm_Module.Robot.Sim.get_state",
}

_DEFAULT_ROBOT_TYPE = os.getenv("FINALPROJECT_ROBOT_TYPE", "sim")


def _load_backend(robot_type: str):
    module_path = _ROBOT_BACKENDS.get(robot_type)
    if not module_path:
        available = ", ".join(_ROBOT_BACKENDS.keys())
        raise ValueError(f"未知 robot_type: {robot_type}，可选: {available}")
    import importlib
    return importlib.import_module(module_path).get_state


def get_state(robot_type: str | None = None) -> dict[str, Any]:
    """获取当前机器人/环境的状态。

    Args:
        robot_type: 机器人类型。默认读取 FINALPROJECT_ROBOT_TYPE（默认 sim）。

    Returns:
        {
            "connected": bool,        是否成功连接
            "robot_type": str,        机器人类型标识
            "observation": {          agent 当前能感知到的一切（自由结构）

                # === 推荐字段（非强制，每个实现自行决定提供哪些） ===
                "agent_position": ...,        agent 在环境中的位置
                "environment": {             环境信息
                    "obstacles": [...],      障碍物/物体
                    "terrain": ...,          地形/场景描述
                },
                "action_result": {           上一步操作的结果
                    "action": str | None,    执行了什么
                    "success": bool | None,  是否成功
                    "feedback": str | None,  反馈信息
                },

                # === 实现特定的自由字段 ===
                ...
            }
        }

    observation 是完全开放的结构，每个 robot 实现自行定义有意义的字段。
    推荐字段只是为了跨实现的通用 agent 提供参考，不是强制要求。
    """
    robot_type = robot_type or _DEFAULT_ROBOT_TYPE
    backend_fn = _load_backend(robot_type)
    return backend_fn()


def list_robot_types() -> list[str]:
    """返回所有可用的 robot_type。"""
    return list(_ROBOT_BACKENDS.keys())


def main() -> int:
    parser = argparse.ArgumentParser(description="获取当前机器人/环境状态。")
    parser.add_argument("--type", type=str, default=None, dest="robot_type",
                        help="机器人类型（默认 sim）")
    parser.add_argument("--json", action="store_true", default=False,
                        help="JSON 格式输出")
    parser.add_argument("--list-types", action="store_true", default=False,
                        help="列出可用 robot_type")
    args = parser.parse_args()

    if args.list_types:
        for rt in list_robot_types():
            print(rt)
        return 0

    try:
        state = get_state(robot_type=args.robot_type)
    except (ValueError, ImportError) as error:
        print(f"错误: {error}", file=sys.stderr)
        return 1

    if args.json:
        print(json.dumps(state, ensure_ascii=False, indent=2))
    else:
        connected = "已连接" if state.get("connected") else "未连接"
        print(f"robot_type:     {state.get('robot_type', '-')}")
        print(f"连接状态:       {connected}")
        if state.get("connected"):
            obs = state.get("observation", {})
            if obs:
                print(f"observation:")
                for key, value in obs.items():
                    if isinstance(value, (dict, list)) and value:
                        print(f"  {key}: {json.dumps(value, ensure_ascii=False, indent=2)}")
                    else:
                        print(f"  {key}: {value}")
            else:
                print("observation:   (空)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
