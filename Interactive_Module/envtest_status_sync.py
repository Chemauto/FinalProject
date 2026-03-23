from __future__ import annotations

import importlib.util
import json
import os
import re
from pathlib import Path
from typing import Any


STATUS_HEADER = "=== EnvTest Live Status ==="
DEFAULT_OBJECT_FACTS_PATH = Path(__file__).resolve().parents[1] / "config" / "object_facts.json"
DEFAULT_CONSTRAINTS = {
    "max_climb_height_m": 0.3,
    "push_only_on_ground": True,
    "climb_requires_adjacency": True,
}
MODEL_USE_TO_SKILL = {
    0: "idle",
    1: "walk",
    2: "climb",
    3: "push_box",
    4: "navigation",
}
DEFAULT_ENVTEST_REPO_ROOT = Path(
    os.getenv("FINALPROJECT_ENVTEST_ROOT", "/home/xcj/work/IsaacLab/IsaacLabBisShe")
)
DEFAULT_CONTROL_FILES = {
    "model_use_file": Path(os.getenv("FINALPROJECT_MODEL_USE_FILE", "/tmp/model_use.txt")),
    "velocity_file": Path(os.getenv("FINALPROJECT_VELOCITY_FILE", "/tmp/envtest_velocity_command.txt")),
    "goal_file": Path(os.getenv("FINALPROJECT_GOAL_FILE", "/tmp/envtest_goal_command.txt")),
    "start_file": Path(os.getenv("FINALPROJECT_START_FILE", "/tmp/envtest_start.txt")),
}
SCENE_OBJECT_SPECS = {
    "left_low_obstacle": {"id": "platform_left_low", "type": "platform", "movable": False},
    "right_low_obstacle": {"id": "platform_right_low", "type": "platform", "movable": False},
    "left_high_obstacle": {"id": "platform_left_high", "type": "platform", "movable": False},
    "right_high_obstacle": {"id": "platform_right_high", "type": "platform", "movable": False},
    "support_box": {"id": "box1", "type": "box", "movable": True},
}


def _default_object_facts_payload() -> dict[str, Any]:
    return {
        "navigation_goal": [0.0, 0.0, 0.0],
        "robot_pose": [0.0, 0.0, 0.0],
        "constraints": dict(DEFAULT_CONSTRAINTS),
        "objects": [],
    }


def _load_raw_object_facts(path: str | Path) -> dict[str, Any]:
    file_path = Path(path)
    if not file_path.exists():
        return _default_object_facts_payload()

    try:
        payload = json.loads(file_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return _default_object_facts_payload()

    if not isinstance(payload, dict):
        return _default_object_facts_payload()

    merged = _default_object_facts_payload()
    merged.update(payload)
    return merged


def _write_object_facts(path: str | Path, payload: dict[str, Any]) -> None:
    file_path = Path(path)
    file_path.parent.mkdir(parents=True, exist_ok=True)
    file_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def _round_list(values: list[float] | None) -> list[float] | None:
    if values is None:
        return None
    return [round(float(value), 3) for value in values]


def _extract_numbers(text: str) -> list[float]:
    return [float(token) for token in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", text)]


def _parse_vector(text: str) -> list[float] | None:
    if text.strip() == "None":
        return None
    values = _extract_numbers(text)
    return values or None


def _read_text_if_exists(path: str | Path) -> str:
    file_path = Path(path)
    if not file_path.exists():
        return ""
    try:
        return file_path.read_text(encoding="utf-8").strip()
    except OSError:
        return ""


def _parse_start(raw_value: str) -> bool | None:
    normalized = raw_value.strip().lower()
    if normalized == "none":
        return None
    if normalized.startswith("1"):
        return True
    if normalized.startswith("0"):
        return False
    return None


def _parse_scalar(raw_value: str) -> int | float | str | None:
    value = raw_value.strip()
    if value == "None":
        return None
    if re.fullmatch(r"[-+]?\d+", value):
        return int(value)
    if re.fullmatch(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", value):
        return float(value)
    return value


def _parse_asset(raw_value: str) -> dict[str, Any] | None:
    value = raw_value.strip()
    if value == "None":
        return None

    name_part, _, remainder = value.partition(",")
    if not remainder:
        return None

    pos_match = re.search(r"pos=\(([^)]*)\)", value)
    size_match = re.search(r"size=\(([^)]*)\)", value)
    position = _parse_vector(pos_match.group(1)) if pos_match else None
    size = _parse_vector(size_match.group(1)) if size_match else None

    return {
        "name": name_part.strip(),
        "position": _round_list(position),
        "size": _round_list(size),
    }


def _read_numeric_file(path: str | Path) -> int | None:
    text = _read_text_if_exists(path)
    if not text:
        return None
    tokens = _extract_numbers(text)
    if not tokens:
        return None
    return int(tokens[0])


def _read_bool_file(path: str | Path) -> bool | None:
    text = _read_text_if_exists(path).lower()
    if not text:
        return None
    if text in {"1", "true", "yes", "on", "run"}:
        return True
    if text in {"0", "false", "no", "off", "idle"}:
        return False
    return None


def _read_vector_file(path: str | Path, valid_lengths: tuple[int, ...]) -> list[float] | None:
    text = _read_text_if_exists(path)
    if not text or text.lower() == "auto":
        return None
    values = _round_list(_parse_vector(text))
    if values is None or len(values) not in valid_lengths:
        return None
    return values


def _iter_process_cmdlines() -> list[tuple[int, list[str]]]:
    processes: list[tuple[int, list[str]]] = []
    for proc_name in os.listdir("/proc"):
        if not proc_name.isdigit():
            continue
        cmdline_path = Path("/proc") / proc_name / "cmdline"
        try:
            raw = cmdline_path.read_bytes()
        except (OSError, PermissionError):
            continue
        if not raw:
            continue
        tokens = [token for token in raw.decode("utf-8", errors="ignore").split("\x00") if token]
        if tokens:
            processes.append((int(proc_name), tokens))
    return processes


def _extract_cli_arg(tokens: list[str], flag: str, default: str | None = None) -> str | None:
    for index, token in enumerate(tokens):
        if token == flag and index + 1 < len(tokens):
            return tokens[index + 1]
        if token.startswith(f"{flag}="):
            return token.split("=", 1)[1]
    return default


def _find_envtest_player_process() -> tuple[int, list[str]] | None:
    candidates: list[tuple[int, list[str]]] = []
    for pid, tokens in _iter_process_cmdlines():
        if any("envtest_model_use_player.py" in token for token in tokens):
            candidates.append((pid, tokens))
    if not candidates:
        return None
    candidates.sort(key=lambda item: item[0], reverse=True)
    return candidates[0]


def _load_scene_layout_module(repo_root: Path):
    scene_layout_path = (
        repo_root
        / "source"
        / "MyProject"
        / "MyProject"
        / "tasks"
        / "manager_based"
        / "EnvTest"
        / "scene_layout.py"
    )
    if not scene_layout_path.exists():
        raise FileNotFoundError(f"未找到 EnvTest scene_layout.py: {scene_layout_path}")

    module_name = "_finalproject_envtest_scene_layout"
    spec = importlib.util.spec_from_file_location(module_name, scene_layout_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"无法加载 EnvTest scene_layout.py: {scene_layout_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _build_scene_objects(scene_id: int, repo_root: Path) -> list[dict[str, Any]]:
    module = _load_scene_layout_module(repo_root)
    scene_layouts = tuple(getattr(module, "SCENE_LAYOUTS", ()))
    active_positions = dict(getattr(module, "ACTIVE_LAYOUT_POSITIONS", {}))
    if not (0 <= scene_id < len(scene_layouts)):
        return []

    layout = scene_layouts[scene_id]
    scene_objects: list[dict[str, Any]] = []
    for asset_name, object_spec in SCENE_OBJECT_SPECS.items():
        if not layout.get(asset_name, False):
            continue
        position = active_positions.get(asset_name)
        size = getattr(module, asset_name.upper().replace("SUPPORT_BOX", "BOX") + "_SIZE", None)
        if asset_name == "support_box":
            size = getattr(module, "BOX_SIZE", size)
        if asset_name == "left_low_obstacle" or asset_name == "right_low_obstacle":
            size = getattr(module, "LOW_OBSTACLE_SIZE", size)
        if asset_name == "left_high_obstacle" or asset_name == "right_high_obstacle":
            size = getattr(module, "HIGH_OBSTACLE_SIZE", size)
        if position is None or size is None:
            continue

        scene_objects.append(
            {
                "id": object_spec["id"],
                "source_name": asset_name,
                "type": object_spec["type"],
                "center": [round(float(position[0]), 3), round(float(position[1]), 3), 0.0],
                "size": [round(float(size[0]), 3), round(float(size[1]), 3), round(float(size[2]), 3)],
                "movable": bool(object_spec["movable"]),
            }
        )
    return scene_objects


def parse_envtest_status_text(text: str) -> dict[str, Any] | None:
    if not text:
        return None

    header_index = text.rfind(STATUS_HEADER)
    if header_index < 0:
        return None

    block = text[header_index:].strip()
    snapshot: dict[str, Any] = {}

    for line in block.splitlines()[1:]:
        line = line.strip()
        if not line or ":" not in line:
            continue

        key, raw_value = line.split(":", 1)
        key = key.strip()
        raw_value = raw_value.strip()

        if key == "start":
            snapshot[key] = _parse_start(raw_value)
        elif key in {"model_use", "scene_id", "unified_obs_dim", "policy_obs_dim"}:
            snapshot[key] = _parse_scalar(raw_value)
        elif key in {"pose_command", "vel_command", "robot_pose", "goal"}:
            snapshot[key] = _round_list(_parse_vector(raw_value))
        elif key in {"platform_1", "platform_2", "box"}:
            snapshot[key] = _parse_asset(raw_value)
        else:
            snapshot[key] = _parse_scalar(raw_value)

    model_use = snapshot.get("model_use")
    if snapshot.get("skill") is None and isinstance(model_use, int):
        snapshot["skill"] = MODEL_USE_TO_SKILL.get(model_use)
    return snapshot or None


def _extract_named_vector(
    user_input: str,
    aliases: tuple[str, ...],
    min_len: int,
    max_len: int,
) -> list[float] | None:
    if not user_input:
        return None

    for alias in aliases:
        pattern = rf"{alias}\s*[:=]\s*(\[[^\]]+\]|\([^\)]+\))"
        match = re.search(pattern, user_input, flags=re.IGNORECASE)
        if not match:
            continue
        values = _round_list(_parse_vector(match.group(1)))
        if values is None or not (min_len <= len(values) <= max_len):
            continue
        return values
    return None


def extract_runtime_overrides(user_input: str) -> dict[str, list[float]]:
    overrides: dict[str, list[float]] = {}

    pose_command = _extract_named_vector(
        user_input,
        aliases=("pose_command", "pose", "位姿指令"),
        min_len=3,
        max_len=4,
    )
    if pose_command is not None:
        overrides["pose_command"] = pose_command

    vel_command = _extract_named_vector(
        user_input,
        aliases=("vel_command", "velocity_command", "vel", "速度指令"),
        min_len=3,
        max_len=3,
    )
    if vel_command is not None:
        overrides["vel_command"] = vel_command

    return overrides


def _build_runtime_objects(snapshot: dict[str, Any]) -> list[dict[str, Any]]:
    runtime_objects: list[dict[str, Any]] = []

    for label in ("platform_1", "platform_2", "box"):
        asset = snapshot.get(label)
        if not isinstance(asset, dict):
            continue

        position = asset.get("position")
        size = asset.get("size")
        if not isinstance(position, list) or len(position) < 3:
            continue
        if not isinstance(size, list) or len(size) < 3:
            continue

        base_z = round(position[2] - size[2] / 2.0, 3)
        runtime_objects.append(
            {
                "id": label,
                "source_name": asset.get("name") or label,
                "type": "box" if label == "box" else "platform",
                "center": [round(position[0], 3), round(position[1], 3), base_z],
                "size": [round(size[0], 3), round(size[1], 3), round(size[2], 3)],
                "movable": label == "box",
            }
        )

    return runtime_objects


def update_object_facts_runtime(
    object_facts_path: str | Path = DEFAULT_OBJECT_FACTS_PATH,
    *,
    snapshot: dict[str, Any] | None = None,
    user_input: str = "",
    scene_objects: list[dict[str, Any]] | None = None,
    replace_objects: bool = False,
) -> dict[str, Any]:
    payload = _load_raw_object_facts(object_facts_path)
    runtime_state = dict(payload.get("runtime_state") or {})

    if snapshot:
        runtime_state.update(snapshot)
        robot_pose = snapshot.get("robot_pose")
        if isinstance(robot_pose, list) and len(robot_pose) >= 3:
            payload["robot_pose"] = _round_list(robot_pose[:3])

        goal = snapshot.get("goal")
        model_use = snapshot.get("model_use")
        if model_use == 4 and isinstance(goal, list) and len(goal) >= 3:
            payload["navigation_goal"] = _round_list(goal[:3])

        runtime_objects = _build_runtime_objects(snapshot)
        if runtime_objects:
            payload["runtime_objects"] = runtime_objects
            if replace_objects:
                payload["objects"] = runtime_objects

    if scene_objects is not None:
        payload["scene_objects"] = scene_objects
        if replace_objects:
            payload["objects"] = scene_objects

    overrides = extract_runtime_overrides(user_input)
    if overrides:
        runtime_state.update(overrides)

    if "pose_command" in runtime_state and isinstance(runtime_state["pose_command"], list):
        runtime_state["pose_command"] = _round_list(runtime_state["pose_command"])
    if "vel_command" in runtime_state and isinstance(runtime_state["vel_command"], list):
        runtime_state["vel_command"] = _round_list(runtime_state["vel_command"])

    payload["runtime_state"] = runtime_state
    _write_object_facts(object_facts_path, payload)
    return payload


def sync_object_facts_from_status_text(
    status_text: str,
    object_facts_path: str | Path = DEFAULT_OBJECT_FACTS_PATH,
    *,
    user_input: str = "",
    replace_objects: bool = False,
) -> dict[str, Any]:
    snapshot = parse_envtest_status_text(status_text)
    if snapshot is None:
        raise ValueError("未在输入文本中找到有效的 EnvTest Live Status 块")
    return update_object_facts_runtime(
        object_facts_path,
        snapshot=snapshot,
        user_input=user_input,
        replace_objects=replace_objects,
    )


def sync_runtime_overrides_from_user_input(
    object_facts_path: str | Path = DEFAULT_OBJECT_FACTS_PATH,
    *,
    user_input: str = "",
) -> dict[str, Any] | None:
    if not extract_runtime_overrides(user_input):
        return None
    return update_object_facts_runtime(object_facts_path, user_input=user_input)


def sync_object_facts_from_live_envtest(
    object_facts_path: str | Path = DEFAULT_OBJECT_FACTS_PATH,
    *,
    user_input: str = "",
) -> dict[str, Any] | None:
    process_info = _find_envtest_player_process()
    if process_info is None:
        return None

    _, tokens = process_info
    repo_root = Path(_extract_cli_arg(tokens, "--repo_root", str(DEFAULT_ENVTEST_REPO_ROOT)) or DEFAULT_ENVTEST_REPO_ROOT)
    scene_id_raw = _extract_cli_arg(tokens, "--scene_id", "0") or "0"
    try:
        scene_id = int(scene_id_raw)
    except ValueError:
        scene_id = 0

    model_use_file = Path(_extract_cli_arg(tokens, "--model_use_file", str(DEFAULT_CONTROL_FILES["model_use_file"])) or DEFAULT_CONTROL_FILES["model_use_file"])
    velocity_file = Path(_extract_cli_arg(tokens, "--velocity_command_file", str(DEFAULT_CONTROL_FILES["velocity_file"])) or DEFAULT_CONTROL_FILES["velocity_file"])
    goal_file = Path(_extract_cli_arg(tokens, "--goal_command_file", str(DEFAULT_CONTROL_FILES["goal_file"])) or DEFAULT_CONTROL_FILES["goal_file"])
    start_file = Path(_extract_cli_arg(tokens, "--start_file", str(DEFAULT_CONTROL_FILES["start_file"])) or DEFAULT_CONTROL_FILES["start_file"])

    snapshot: dict[str, Any] = {
        "scene_id": scene_id,
        "model_use": _read_numeric_file(model_use_file),
        "start": _read_bool_file(start_file),
        "vel_command": _read_vector_file(velocity_file, (3,)),
        "pose_command": _read_vector_file(goal_file, (3, 4)),
        "goal": _read_vector_file(goal_file, (3, 4)),
        "robot_pose": [0.0, 0.0, 0.0],
    }
    if isinstance(snapshot.get("model_use"), int):
        snapshot["skill"] = MODEL_USE_TO_SKILL.get(int(snapshot["model_use"]))

    scene_objects = _build_scene_objects(scene_id, repo_root)
    return update_object_facts_runtime(
        object_facts_path,
        snapshot=snapshot,
        user_input=user_input,
        scene_objects=scene_objects,
        replace_objects=True,
    )
