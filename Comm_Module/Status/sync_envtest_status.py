#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    from .envtest_status_sync import (
        DEFAULT_OBJECT_FACTS_PATH,
        sync_object_facts_from_live_envtest,
        sync_object_facts_from_status_text,
    )
except ImportError:
    from envtest_status_sync import (
        DEFAULT_OBJECT_FACTS_PATH,
        sync_object_facts_from_live_envtest,
        sync_object_facts_from_status_text,
    )


def _read_status_text(args: argparse.Namespace) -> str:
    if args.status_text:
        return args.status_text
    if args.status_file:
        return Path(args.status_file).read_text(encoding="utf-8")
    return sys.stdin.read()


def main() -> int:
    parser = argparse.ArgumentParser(description="把 EnvTest Live Status 同步到 object_facts.json。")
    parser.add_argument(
        "--object-facts",
        type=str,
        default=str(DEFAULT_OBJECT_FACTS_PATH),
        help="object_facts.json 路径。",
    )
    parser.add_argument(
        "--status-file",
        type=str,
        default="",
        help="包含 EnvTest Live Status 文本块的文件路径。",
    )
    parser.add_argument(
        "--status-text",
        type=str,
        default="",
        help="直接传入 EnvTest Live Status 文本块。",
    )
    parser.add_argument(
        "--user-input",
        type=str,
        default="",
        help="可选用户输入；若包含 pose_command=[...] / vel_command=[...]，会覆盖 runtime_state 中对应字段。",
    )
    parser.add_argument(
        "--replace-objects",
        action="store_true",
        default=False,
        help="如果启用，则用状态块里的 platform_1/platform_2/box 替换 top-level objects。",
    )
    parser.add_argument(
        "--live-envtest",
        action="store_true",
        default=False,
        help="直接从正在运行的 envtest_model_use_player.py 进程和控制文件同步。",
    )
    args = parser.parse_args()

    if args.live_envtest:
        payload = sync_object_facts_from_live_envtest(
            object_facts_path=args.object_facts,
            user_input=args.user_input,
        )
        if payload is None:
            print("未发现正在运行的 envtest_model_use_player.py 进程。", file=sys.stderr)
            return 1
    else:
        status_text = _read_status_text(args).strip()
        if not status_text:
            print("未提供 EnvTest Live Status 文本。", file=sys.stderr)
            return 1

        payload = sync_object_facts_from_status_text(
            status_text=status_text,
            object_facts_path=args.object_facts,
            user_input=args.user_input,
            replace_objects=args.replace_objects,
        )
    runtime_state = payload.get("runtime_state") or {}
    print(f"[sync_envtest_status] 已更新: {args.object_facts}")
    print(f"[sync_envtest_status] runtime_state.model_use={runtime_state.get('model_use')}")
    print(f"[sync_envtest_status] runtime_state.skill={runtime_state.get('skill')}")
    print(f"[sync_envtest_status] runtime_state.pose_command={runtime_state.get('pose_command')}")
    print(f"[sync_envtest_status] runtime_state.vel_command={runtime_state.get('vel_command')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
