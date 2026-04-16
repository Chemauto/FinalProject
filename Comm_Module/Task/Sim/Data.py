#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""IsaacLab EnvTest 的后端定义与状态格式声明。"""

from __future__ import annotations

import json

from Comm_Module.Task import TaskBackend


STATE_SCHEMA = {
    "connected": {
        "source": "connected",
        "default": False,
    },
    "task_type": {
        "source": "task_type",
        "default": "sim",
    },
    "robot_type": {
        "source": "task_type",
        "default": "sim",
    },
    "observation": {
        "agent_position": {
            "source": "snapshot.robot_pose",
            "default": None,
        },
        "environment": {
            "scene_id": {
                "source": "snapshot.scene_id",
                "default": None,
            },
            "obstacles": {
                "source": "scene_objects",
                "default": [],
            },
            "goal": {
                "source": "snapshot.goal",
                "default": None,
            },
        },
        "action_result": {
            "skill": {
                "source": "snapshot.skill",
                "default": None,
            },
            "model_use": {
                "source": "snapshot.model_use",
                "default": None,
            },
            "vel_command": {
                "source": "snapshot.vel_command",
                "default": None,
            },
            "start": {
                "source": "snapshot.start",
                "default": None,
            },
        },
        "raw": {
            "snapshot": {
                "source": "snapshot",
                "default": {},
            },
            "scene_objects": {
                "source": "scene_objects",
                "default": [],
            },
            "pose_command": {
                "source": "snapshot.pose_command",
                "default": None,
            },
            "status_file_available": {
                "source": "status_file_available",
                "default": False,
            },
            "process": {
                "source": "process",
                "default": None,
            },
            "control_files": {
                "source": "control_files",
                "default": {},
            },
            "repo_root": {
                "source": "repo_root",
                "default": None,
            },
        },
    },
    "runtime": {
        "timestamp": {
            "source": "snapshot.timestamp",
            "default": None,
        },
        "snapshot": {
            "source": "snapshot",
            "default": {},
        },
        "skill": {
            "source": "snapshot.skill",
            "default": None,
        },
        "model_use": {
            "source": "snapshot.model_use",
            "default": None,
        },
        "goal": {
            "source": "snapshot.goal",
            "default": None,
        },
        "start": {
            "source": "snapshot.start",
            "default": None,
        },
        "scene_objects": {
            "source": "scene_objects",
            "default": [],
        },
    },
}


TASK_DATA = TaskBackend(
    task_type="sim",
    description="IsaacLab EnvTest 仿真任务后端",
    data_module="Comm_Module.Task.Sim.get_data",
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
        "data_getter_name": TASK_DATA.data_getter_name,
        "state_schema": TASK_DATA.state_schema,
    }
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
