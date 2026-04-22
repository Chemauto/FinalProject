#!/usr/bin/env python3
"""Relay FinalProject /go2/cmd_vel commands to legged_ros2 /rl_cmd_vel.

This is the minimal real-robot bridge for velocity-only testing. It does not
modify legged_ros2 controller defaults and it does not publish status or odom.
Run it only when FinalProject should drive the already-active rl_controller.
"""

from __future__ import annotations

import argparse
from typing import Any

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    ROS2_AVAILABLE = True
except Exception:  # pragma: no cover - exercised only on machines without ROS2.
    rclpy = None
    Node = object
    Twist = None
    DurabilityPolicy = HistoryPolicy = QoSProfile = ReliabilityPolicy = None
    ROS2_AVAILABLE = False


def clamp_value(value: float, limit: float) -> float:
    """Clamp value to +/- limit. A non-positive limit disables clamping."""
    numeric = float(value)
    bound = abs(float(limit))
    if bound <= 0.0:
        return numeric
    return max(-bound, min(bound, numeric))


def normalize_qos_reliability(value: str) -> str:
    """Normalize user-facing QoS reliability strings."""
    text = str(value or "").strip().lower().replace("-", "_")
    if text in {"reliable", "reliability_reliable"}:
        return "reliable"
    if text in {"best_effort", "besteffort", "best", "sensor_data", "reliability_best_effort"}:
        return "best_effort"
    return "best_effort"


def make_input_qos(reliability: str) -> Any:
    """Build the input subscription QoS profile."""
    if QoSProfile is None:
        return 10
    normalized = normalize_qos_reliability(reliability)
    reliability_policy = (
        ReliabilityPolicy.RELIABLE
        if normalized == "reliable"
        else ReliabilityPolicy.BEST_EFFORT
    )
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=reliability_policy,
        durability=DurabilityPolicy.VOLATILE,
    )


def twist_to_dict(msg: Any) -> dict[str, list[float]]:
    """Extract linear and angular vectors from a Twist-like message."""
    return {
        "linear": [
            float(msg.linear.x),
            float(msg.linear.y),
            float(msg.linear.z),
        ],
        "angular": [
            float(msg.angular.x),
            float(msg.angular.y),
            float(msg.angular.z),
        ],
    }


def copy_twist_fields(
    command: dict[str, list[float]],
    *,
    max_linear: float = 0.0,
    max_angular: float = 0.0,
) -> dict[str, list[float]]:
    """Return a relay-safe command dict, optionally clamping linear/angular axes."""
    linear = list(command.get("linear") or [0.0, 0.0, 0.0])
    angular = list(command.get("angular") or [0.0, 0.0, 0.0])
    while len(linear) < 3:
        linear.append(0.0)
    while len(angular) < 3:
        angular.append(0.0)
    return {
        "linear": [clamp_value(value, max_linear) for value in linear[:3]],
        "angular": [clamp_value(value, max_angular) for value in angular[:3]],
    }


def dict_to_twist(command: dict[str, list[float]]) -> Any:
    """Build a geometry_msgs/Twist from a command dict."""
    msg = Twist()
    linear = command["linear"]
    angular = command["angular"]
    msg.linear.x = float(linear[0])
    msg.linear.y = float(linear[1])
    msg.linear.z = float(linear[2])
    msg.angular.x = float(angular[0])
    msg.angular.y = float(angular[1])
    msg.angular.z = float(angular[2])
    return msg


class Go2CmdVelRelay(Node):
    """ROS2 node that relays /go2/cmd_vel to /rl_cmd_vel."""

    def __init__(self, args: argparse.Namespace):
        super().__init__("go2_cmd_vel_relay")
        self.args = args
        self.publisher = self.create_publisher(Twist, args.output_topic, 10)
        self.create_subscription(Twist, args.input_topic, self._on_cmd_vel, make_input_qos(args.input_qos))
        self.get_logger().info(
            f"Relaying {args.input_topic} -> {args.output_topic} "
            f"(input_qos={normalize_qos_reliability(args.input_qos)}, "
            f"max_linear={args.max_linear_speed}, max_angular={args.max_angular_speed})"
        )

    def _on_cmd_vel(self, msg: Any) -> None:
        command = copy_twist_fields(
            twist_to_dict(msg),
            max_linear=self.args.max_linear_speed,
            max_angular=self.args.max_angular_speed,
        )
        self.publisher.publish(dict_to_twist(command))
        if self.args.verbose:
            linear = command["linear"]
            angular = command["angular"]
            self.get_logger().info(
                "cmd_vel -> "
                f"linear=({linear[0]:.3f}, {linear[1]:.3f}, {linear[2]:.3f}), "
                f"angular=({angular[0]:.3f}, {angular[1]:.3f}, {angular[2]:.3f})"
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Relay FinalProject /go2/cmd_vel to legged_ros2 /rl_cmd_vel.")
    parser.add_argument("--input-topic", default="/go2/cmd_vel", help="Input Twist topic from FinalProject.")
    parser.add_argument("--output-topic", default="/rl_cmd_vel", help="Output Twist topic consumed by legged_ros2.")
    parser.add_argument(
        "--input-qos",
        choices=("best_effort", "reliable"),
        default="best_effort",
        help="QoS reliability for /go2/cmd_vel subscription. best_effort is most compatible.",
    )
    parser.add_argument(
        "--max-linear-speed",
        type=float,
        default=0.0,
        help="Optional absolute clamp for linear x/y/z. 0 disables clamping.",
    )
    parser.add_argument(
        "--max-angular-speed",
        type=float,
        default=0.0,
        help="Optional absolute clamp for angular x/y/z. 0 disables clamping.",
    )
    parser.add_argument("--verbose", action="store_true", help="Log relayed commands.")
    return parser.parse_args()


def main() -> int:
    if not ROS2_AVAILABLE:
        raise SystemExit("ROS2 Python packages are not available. Source your ROS2 environment first.")

    args = parse_args()
    rclpy.init()
    node = Go2CmdVelRelay(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("go2_cmd_vel_relay stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
