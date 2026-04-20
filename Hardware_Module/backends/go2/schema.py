#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unitree Go2 真机后端定义与状态格式声明。"""

from __future__ import annotations

import json

from Hardware_Module.registry import TaskBackend


# ── ROS2 topic 名字（TODO: 替换为实际 topic） ─────────────────────────
ROS2_TOPICS = {
    "robot_state": "/go2/state",
    "odom": "/go2/odom",
    "skill_status": "/go2/skill_status",
    "scene_objects": "/go2/scene_objects",
    "camera": "/go2/wrist_camera/image_raw",
}


STATE_SCHEMA = {
    "connected": {
        "source": "connected",
        "default": False,
    },
    "task_type": {
        "source": "task_type",
        "default": "go2",
    },
    "robot_type": {
        "source": "task_type",
        "default": "go2",
    },
    "observation": {
        "agent_position": {
            "source": "odom.position",
            "default": None,
        },
        "environment": {
            "scene_id": {
                "source": "scene_id",
                "default": None,
            },
            "obstacles": {
                "source": "scene_objects",
                "default": [],
            },
            "goal": {
                "source": "skill_status.goal",
                "default": None,
            },
        },
        "action_result": {
            "skill": {
                "source": "skill_status.skill",
                "default": None,
            },
            "model_use": {
                "source": "skill_status.model_use",
                "default": None,
            },
            "vel_command": {
                "source": "skill_status.vel_command",
                "default": None,
            },
            "start": {
                "source": "skill_status.start",
                "default": None,
            },
        },
    },
    "runtime": {
        "timestamp": {
            "source": "timestamp",
            "default": None,
        },
        "skill": {
            "source": "skill_status.skill",
            "default": None,
        },
        "model_use": {
            "source": "skill_status.model_use",
            "default": None,
        },
        "goal": {
            "source": "skill_status.goal",
            "default": None,
        },
        "start": {
            "source": "skill_status.start",
            "default": None,
        },
        "scene_objects": {
            "source": "scene_objects",
            "default": [],
        },
    },
}


TASK_DATA = TaskBackend(
    task_type="go2",
    description="Unitree Go2 真机 ROS2 后端",
    data_module="Hardware_Module.backends.go2.data",
    state_schema=STATE_SCHEMA,
    data_getter_name="get_data",
    sync_object_facts_from_live_data_name="sync_object_facts_from_live_data",
    sync_runtime_overrides_from_user_input_name="sync_runtime_overrides_from_user_input",
)


def main() -> int:
    payload = {
        "task_type": TASK_DATA.task_type,
        "description": TASK_DATA.description,
        "data_module": TASK_DATA.data_module,
        "ros2_topics": ROS2_TOPICS,
        "state_schema": TASK_DATA.state_schema,
    }
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
