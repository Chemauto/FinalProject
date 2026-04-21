#!/usr/bin/env python3
"""Bridge FinalProject Go2 ROS2 topics to IsaacLab EnvTest Sim2Sim controls.

The bridge is intentionally small:
- subscribe to FinalProject command topics under /go2/*
- forward commands to IsaacLabBisShe Socket/envtest_socket_server.py over UDP
- poll /tmp/envtest_live_status.json
- republish it as /go2/odom, /go2/skill_status, and /go2/scene_objects

Pure conversion helpers are kept at module level so they can be tested without ROS2.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
from pathlib import Path
from typing import Any, Iterable

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import String

    ROS2_AVAILABLE = True
except Exception:  # pragma: no cover - exercised only on machines without ROS2.
    rclpy = None
    Node = object
    PoseStamped = Twist = Odometry = String = None
    ROS2_AVAILABLE = False


SKILL_NAME_TO_MODEL_USE = {
    "idle": 0,
    "walk": 1,
    "climb": 2,
    "push_box": 3,
    "navigation": 4,
    "nav": 4,
    "climb_align": 4,
    "nav_climb": 5,
}


def _float_list(values: Iterable[Any], max_len: int | None = None) -> list[float]:
    items = list(values)
    if max_len is not None:
        items = items[:max_len]
    return [float(value) for value in items]


def _format_vector(values: Iterable[Any], *, max_len: int = 3) -> str:
    return ",".join(str(float(value)) for value in list(values)[:max_len])


def _bool_to_start(value: Any) -> int:
    if isinstance(value, str):
        return 1 if value.strip().lower() in {"1", "true", "yes", "y", "on", "run"} else 0
    return 1 if bool(value) else 0


def skill_command_to_udp_text(payload: dict[str, Any]) -> str:
    """Convert /go2/skill_command JSON payload into EnvTest UDP text."""
    fields: list[str] = []
    model_use = payload.get("model_use")
    if model_use is None and payload.get("skill") is not None:
        model_use = SKILL_NAME_TO_MODEL_USE.get(str(payload["skill"]).strip().lower())
    if model_use is not None:
        fields.append(f"model_use={int(model_use)}")

    velocity = payload.get("velocity") or payload.get("vel_command")
    if velocity is not None:
        fields.append(f"velocity={_format_vector(velocity)}")

    goal = payload.get("goal")
    if goal is not None:
        fields.append("goal=auto" if goal == "auto" else f"goal={_format_vector(goal)}")

    if "start" in payload:
        fields.append(f"start={_bool_to_start(payload.get('start'))}")

    if payload.get("reset") is not None:
        fields.append(f"reset={int(payload['reset'])}")

    return "; ".join(fields)


def twist_to_velocity(msg: Any) -> list[float]:
    linear = msg.linear
    return [float(linear.x), float(linear.y), float(linear.z)]


def _pose_from_message(msg: Any) -> Any:
    pose = getattr(msg, "pose", None)
    if pose is not None and hasattr(pose, "pose"):
        return pose.pose
    return pose


def pose_to_goal(msg: Any) -> list[float]:
    pose = _pose_from_message(msg)
    position = pose.position
    orientation = pose.orientation
    yaw = math.atan2(
        2.0 * (float(orientation.w) * float(orientation.z) + float(orientation.x) * float(orientation.y)),
        1.0 - 2.0 * (float(orientation.y) ** 2 + float(orientation.z) ** 2),
    )
    return [float(position.x), float(position.y), float(position.z), round(yaw, 6)]


def read_status_snapshot(path: str | Path) -> dict[str, Any] | None:
    file_path = Path(path)
    if not file_path.exists():
        return None
    try:
        payload = json.loads(file_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def _asset_to_alignment(asset: dict[str, Any] | None) -> dict[str, Any] | None:
    if not isinstance(asset, dict):
        return None
    return {
        "name": asset.get("name"),
        "position": _float_list(asset.get("position") or [], max_len=3),
        "size": _float_list(asset.get("size") or [], max_len=3),
    }


def build_envtest_alignment(snapshot: dict[str, Any]) -> dict[str, Any]:
    alignment = {}
    for key in ("platform_1", "platform_2", "box"):
        asset = _asset_to_alignment(snapshot.get(key))
        if asset is not None:
            alignment[key] = asset
    return alignment


def build_skill_status(snapshot: dict[str, Any]) -> dict[str, Any]:
    status = {
        "timestamp": snapshot.get("timestamp"),
        "model_use": snapshot.get("model_use"),
        "skill": snapshot.get("skill"),
        "scene_id": snapshot.get("scene_id"),
        "start": snapshot.get("start"),
        "goal": snapshot.get("goal"),
        "vel_command": snapshot.get("vel_command"),
    }
    alignment = build_envtest_alignment(snapshot)
    if alignment:
        status["envtest_alignment"] = alignment
    return status


def _asset_to_scene_object(asset: dict[str, Any] | None, object_type: str) -> dict[str, Any] | None:
    if not isinstance(asset, dict):
        return None
    name = str(asset.get("name") or object_type)
    position = asset.get("position")
    size = asset.get("size")
    if not isinstance(position, list) or not isinstance(size, list):
        return None
    return {
        "id": name,
        "type": object_type,
        "center": _float_list(position, max_len=3),
        "size": _float_list(size, max_len=3),
        "movable": object_type == "box",
    }


def build_scene_objects(snapshot: dict[str, Any]) -> list[dict[str, Any]]:
    objects: list[dict[str, Any]] = []
    for key in ("platform_1", "platform_2"):
        obj = _asset_to_scene_object(snapshot.get(key), "platform")
        if obj is not None:
            objects.append(obj)
    box = _asset_to_scene_object(snapshot.get("box"), "box")
    if box is not None:
        objects.append(box)
    return objects


class UdpCommandSender:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = int(port)

    def send_text(self, text: str) -> None:
        message = text.strip()
        if not message:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.sendto(message.encode("utf-8"), (self.host, self.port))
        finally:
            sock.close()


class Go2SkillBridge(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("go2_skill_bridge")
        self.args = args
        self.sender = UdpCommandSender(args.udp_host, args.udp_port)
        self.status_path = Path(args.status_json)

        self.odom_pub = self.create_publisher(Odometry, args.odom_topic, 10)
        self.skill_status_pub = self.create_publisher(String, args.skill_status_topic, 10)
        self.scene_objects_pub = self.create_publisher(String, args.scene_objects_topic, 10)

        self.create_subscription(String, args.skill_command_topic, self._on_skill_command, 10)
        self.create_subscription(Twist, args.cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_subscription(PoseStamped, args.goal_pose_topic, self._on_goal_pose, 10)

        period = 1.0 / max(float(args.publish_hz), 0.1)
        self.create_timer(period, self._publish_status_snapshot)
        self.get_logger().info(
            f"Bridge ready: /go2 commands -> UDP {args.udp_host}:{args.udp_port}, status <- {self.status_path}"
        )

    def _send_udp(self, text: str) -> None:
        if not text:
            return
        self.sender.send_text(text)
        if self.args.verbose:
            self.get_logger().info(f"UDP -> {text}")

    def _on_skill_command(self, msg: Any) -> None:
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as exc:
            self.get_logger().warn(f"Invalid skill_command JSON: {exc}")
            return
        if not isinstance(payload, dict):
            self.get_logger().warn("Invalid skill_command payload: expected object")
            return
        self._send_udp(skill_command_to_udp_text(payload))

    def _on_cmd_vel(self, msg: Any) -> None:
        velocity = twist_to_velocity(msg)
        self._send_udp(f"velocity={_format_vector(velocity)}")

    def _on_goal_pose(self, msg: Any) -> None:
        goal = pose_to_goal(msg)
        self._send_udp(f"goal={_format_vector(goal)}")

    def _publish_status_snapshot(self) -> None:
        snapshot = read_status_snapshot(self.status_path)
        if snapshot is None:
            return
        self._publish_odom(snapshot)
        self._publish_json(self.skill_status_pub, build_skill_status(snapshot))
        self._publish_json(self.scene_objects_pub, build_scene_objects(snapshot))

    def _publish_odom(self, snapshot: dict[str, Any]) -> None:
        pose = snapshot.get("robot_pose")
        if not isinstance(pose, list) or len(pose) < 3:
            return
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.frame_id
        msg.child_frame_id = self.args.child_frame_id
        msg.pose.pose.position.x = float(pose[0])
        msg.pose.pose.position.y = float(pose[1])
        msg.pose.pose.position.z = float(pose[2])
        msg.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(msg)

    @staticmethod
    def _publish_json(publisher: Any, payload: Any) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        publisher.publish(msg)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Bridge FinalProject Go2 ROS2 topics to IsaacLab EnvTest Sim2Sim.")
    parser.add_argument("--udp-host", default="127.0.0.1", help="EnvTest UDP server host.")
    parser.add_argument("--udp-port", type=int, default=5566, help="EnvTest UDP server port.")
    parser.add_argument("--status-json", default="/tmp/envtest_live_status.json", help="EnvTest live status JSON path.")
    parser.add_argument("--publish-hz", type=float, default=10.0, help="Status publish frequency.")
    parser.add_argument("--skill-command-topic", default="/go2/skill_command")
    parser.add_argument("--cmd-vel-topic", default="/go2/cmd_vel")
    parser.add_argument("--goal-pose-topic", default="/go2/goal_pose")
    parser.add_argument("--odom-topic", default="/go2/odom")
    parser.add_argument("--skill-status-topic", default="/go2/skill_status")
    parser.add_argument("--scene-objects-topic", default="/go2/scene_objects")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--child-frame-id", default="base")
    parser.add_argument("--verbose", action="store_true", help="Log forwarded UDP commands.")
    return parser.parse_args()


def main() -> int:
    if not ROS2_AVAILABLE:
        raise SystemExit("ROS2 Python packages are not available. Source your ROS2 environment first.")

    args = parse_args()
    rclpy.init()
    node = Go2SkillBridge(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
