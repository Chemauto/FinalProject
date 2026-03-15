"""
四足导航技能模块 (Navigation Skills Module)

当前只暴露 4 个技能给 LLM：
1. walk
2. climb
3. push_box
4. way_select

其中 way_select 是路线选择技能，内部会调用底层 walk，
用于机器人从中间位置切换到左侧或右侧路线入口。

当前版本仍是 demo，只打印语音播报和技能调用信息。
"""

from __future__ import annotations

import argparse
import json
import sys
from typing import Any

CLIMB_LIMIT_METERS = 0.3
DEFAULT_TARGET = "前方目标点"


def _format_height(height: float) -> str:
    return f"{height:.2f}"


def _speak(text: str) -> None:
    if text:
        print(f"[go2.speech] {text}", file=sys.stderr)


def _log_skill(skill_name: str, detail: str) -> None:
    print(f"[go2.skill] {skill_name}: {detail}", file=sys.stderr)


def _normalize_direction(direction: str) -> str:
    normalized = direction.strip().lower()
    mapping = {
        "left": "left",
        "right": "right",
        "左": "left",
        "右": "right",
        "左边": "left",
        "右边": "right",
        "左侧": "left",
        "右侧": "right",
    }
    if normalized not in mapping:
        raise ValueError("direction 只能是 left/right 或 左/右")
    return mapping[normalized]


async def execute_walk_skill(
    route_side: str = "前方",
    distance: float = 1.0,
    target: str = DEFAULT_TARGET,
    speech: str = "",
) -> dict[str, Any]:
    """行走技能 demo。"""
    if distance <= 0:
        raise ValueError("distance 必须大于 0")

    _speak(speech)
    _log_skill("walk", f"沿{route_side}路径行走 {distance:.2f} 米，目标={target}")
    return {
        "skill": "walk",
        "route_side": route_side,
        "distance": distance,
        "target": target,
        "speech": speech,
        "backend": "demo_print",
        "status": "success",
    }


async def execute_climb_skill(
    height: float,
    stage: str = "高台",
    target: str = DEFAULT_TARGET,
    speech: str = "",
) -> dict[str, Any]:
    """攀爬技能 demo。"""
    if height <= 0:
        raise ValueError("height 必须大于 0")
    if height > CLIMB_LIMIT_METERS:
        raise ValueError(
            f"当前 demo 约束为最大攀爬高度 {CLIMB_LIMIT_METERS:.2f} 米，收到 {height:.2f} 米"
        )

    _speak(speech)
    _log_skill("climb", f"攀爬{stage}，高度 {height:.2f} 米，目标={target}")
    return {
        "skill": "climb",
        "height": height,
        "stage": stage,
        "target": target,
        "speech": speech,
        "backend": "demo_print",
        "status": "success",
    }


async def execute_push_box_skill(
    box_height: float,
    target_position: str = "高台旁边",
    speech: str = "",
) -> dict[str, Any]:
    """推箱子技能 demo。"""
    if box_height <= 0:
        raise ValueError("box_height 必须大于 0")

    _speak(speech)
    _log_skill("push_box", f"推动高度 {box_height:.2f} 米的箱子到{target_position}")
    return {
        "skill": "push_box",
        "box_height": box_height,
        "target_position": target_position,
        "speech": speech,
        "backend": "demo_print",
        "status": "success",
    }


async def execute_way_select_skill(
    direction: str,
    lateral_distance: float = 0.5,
    target: str = DEFAULT_TARGET,
    speech: str = "",
) -> dict[str, Any]:
    """路线选择技能 demo，内部调用 walk 完成横向切换。"""
    normalized_direction = _normalize_direction(direction)
    direction_label = "左侧" if normalized_direction == "left" else "右侧"

    if lateral_distance <= 0:
        raise ValueError("lateral_distance 必须大于 0")

    if not speech:
        speech = f"机器人当前位于中间位置，先切换到{direction_label}路线。"

    _speak(speech)
    _log_skill("way_select", f"选择{direction_label}路线，调用底层 walk 完成路线切换")

    base_walk = await execute_walk_skill(
        route_side=f"{direction_label}路线入口",
        distance=lateral_distance,
        target=f"{direction_label}路线入口",
        speech="",
    )

    return {
        "skill": "way_select",
        "direction": normalized_direction,
        "lateral_distance": lateral_distance,
        "target": target,
        "speech": speech,
        "base_skill_call": base_walk,
        "backend": "demo_print",
        "status": "success",
    }


async def execute_case_2_flow(
    target: str = DEFAULT_TARGET,
    left_height: float = 0.2,
    right_height: float = 0.0,
    lateral_distance: float = 0.5,
    forward_distance: float = 1.0,
) -> dict[str, Any]:
    """case 2 本地 demo：右侧绕行。"""
    if left_height <= 0 or left_height > CLIMB_LIMIT_METERS:
        raise ValueError("case 2 要求左侧高台高度在 (0, 0.3] 米范围内")
    if right_height > 0.05:
        raise ValueError("case 2 要求右侧无障碍或近似无障碍")

    speech = (
        f"前方左侧检测到高台，高度约 {_format_height(left_height)} 米，"
        f"右侧障碍高度约 {_format_height(right_height)} 米。"
        f"按照简单快速原则，先选择右侧路线，再调用行走技能前往{target}。"
    )

    execution = [
        await execute_way_select_skill(
            direction="right",
            lateral_distance=lateral_distance,
            target=target,
            speech=speech,
        ),
        await execute_walk_skill(
            route_side="右侧",
            distance=forward_distance,
            target=target,
            speech=f"已进入右侧路线，继续行走到{target}。",
        ),
    ]

    return {
        "status": "success",
        "scene_type": "case_2_left_platform_right_clear",
        "target": target,
        "observation": {
            "left_height": left_height,
            "right_height": right_height,
            "climb_limit": CLIMB_LIMIT_METERS,
        },
        "execution": execution,
    }


async def execute_case_4_flow(
    target: str = DEFAULT_TARGET,
    left_height: float = 0.4,
    right_height: float = 0.4,
    box_height: float = 0.2,
    lateral_distance: float = 0.5,
    forward_distance: float = 1.0,
    box_side: str = "left",
) -> dict[str, Any]:
    """case 4 本地 demo：选择箱子路线，推箱子后二段攀爬。"""
    if left_height <= CLIMB_LIMIT_METERS or right_height <= CLIMB_LIMIT_METERS:
        raise ValueError("case 4 要求左右高台都超过最大攀爬高度")
    if box_height <= 0 or box_height > CLIMB_LIMIT_METERS:
        raise ValueError("case 4 要求箱子高度在 (0, 0.3] 米范围内")

    climb_from_box_height = max(left_height, right_height) - box_height
    if climb_from_box_height <= 0 or climb_from_box_height > CLIMB_LIMIT_METERS:
        raise ValueError("case 4 的平台高度和箱子高度组合不满足二段攀爬条件")

    box_side_label = "左侧" if _normalize_direction(box_side) == "left" else "右侧"
    speech = (
        f"前方左右两侧均检测到高台，左侧高度约 {_format_height(left_height)} 米，"
        f"右侧高度约 {_format_height(right_height)} 米，超过最大攀爬高度 "
        f"{_format_height(CLIMB_LIMIT_METERS)} 米。检测到{box_side_label}存在可推动箱子，"
        f"高度约 {_format_height(box_height)} 米。按照简单快速原则，先选择有箱子的路线，"
        f"再执行推箱子和攀爬动作前往{target}。"
    )

    execution = [
        await execute_way_select_skill(
            direction=box_side,
            lateral_distance=lateral_distance,
            target=target,
            speech=speech,
        ),
        await execute_push_box_skill(
            box_height=box_height,
            target_position="高台旁边",
            speech="开始推动箱子到高台旁边。",
        ),
        await execute_climb_skill(
            height=box_height,
            stage="箱子顶部",
            target=target,
            speech="箱子已到位，先攀爬到箱子顶部。",
        ),
        await execute_climb_skill(
            height=climb_from_box_height,
            stage="箱子到高台的二段攀爬",
            target=target,
            speech="继续从箱子顶部攀爬到高台。",
        ),
        await execute_walk_skill(
            route_side="高台上方",
            distance=forward_distance,
            target=target,
            speech=f"已经到达高台，继续行走到{target}。",
        ),
    ]

    return {
        "status": "success",
        "scene_type": "case_4_high_platform_with_box",
        "target": target,
        "observation": {
            "left_height": left_height,
            "right_height": right_height,
            "box_height": box_height,
            "box_side": _normalize_direction(box_side),
            "climb_limit": CLIMB_LIMIT_METERS,
        },
        "execution": execution,
    }


def register_tools(mcp):
    """注册 4 个导航技能。"""

    @mcp.tool()
    async def walk(
        route_side: str = "前方",
        distance: float = 1.0,
        target: str = DEFAULT_TARGET,
        speech: str = "",
    ) -> str:
        """行走技能。

        用于机器人沿当前路线继续前进。
        当前版本只打印技能执行信息。
        """
        result = await execute_walk_skill(
            route_side=route_side,
            distance=distance,
            target=target,
            speech=speech,
        )
        return json.dumps(result, ensure_ascii=False)

    @mcp.tool()
    async def climb(
        height: float,
        stage: str = "高台",
        target: str = DEFAULT_TARGET,
        speech: str = "",
    ) -> str:
        """攀爬技能。

        用于机器人攀爬箱子或高台。
        当前版本只打印技能执行信息。
        """
        result = await execute_climb_skill(
            height=height,
            stage=stage,
            target=target,
            speech=speech,
        )
        return json.dumps(result, ensure_ascii=False)

    @mcp.tool()
    async def push_box(
        box_height: float,
        target_position: str = "高台旁边",
        speech: str = "",
    ) -> str:
        """推箱子技能。

        用于把可推动箱子移动到高台旁边。
        当前版本只打印技能执行信息。
        """
        result = await execute_push_box_skill(
            box_height=box_height,
            target_position=target_position,
            speech=speech,
        )
        return json.dumps(result, ensure_ascii=False)

    @mcp.tool()
    async def way_select(
        direction: str,
        lateral_distance: float = 0.5,
        target: str = DEFAULT_TARGET,
        speech: str = "",
    ) -> str:
        """路线选择技能。

        机器人初始位于中间位置时，优先调用该技能切换到左侧或右侧路线。
        内部会调用底层 walk 完成横向移动。
        """
        result = await execute_way_select_skill(
            direction=direction,
            lateral_distance=lateral_distance,
            target=target,
            speech=speech,
        )
        return json.dumps(result, ensure_ascii=False)

    print("[navigation_demo.py:register_tools] 导航技能模块已注册 (4 个工具)", file=sys.stderr)

    return {
        "walk": walk,
        "climb": climb,
        "push_box": push_box,
        "way_select": way_select,
    }


async def _run_cli_demo(scene: str) -> dict[str, Any]:
    """命令行 demo 入口。"""
    if scene == "case2":
        return await execute_case_2_flow()
    if scene == "case4":
        return await execute_case_4_flow()
    raise ValueError(f"不支持的场景: {scene}")


def main() -> None:
    """支持直接运行模块做本地场景演示。"""
    parser = argparse.ArgumentParser(description="四足导航技能演示")
    parser.add_argument("scene", choices=["case2", "case4"], help="要运行的演示场景")
    args = parser.parse_args()

    import asyncio

    result = asyncio.run(_run_cli_demo(args.scene))
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
