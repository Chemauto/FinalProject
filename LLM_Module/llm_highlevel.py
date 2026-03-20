from __future__ import annotations

import json
import re
from pathlib import Path

import yaml


class HighLevelPlanner:
    """上层 LLM：负责把用户输入拆成任务序列。"""

    _OBSTACLE_KEYWORDS = ("高台", "矮墙", "长凳", "障碍", "箱子", "台阶", "结构", "墙体", "混凝土")
    _NAVIGATION_KEYWORDS = ("前往", "目标点", "到目标点", "到达", "导航")

    def __init__(self, client, model: str, prompt_path: str | None = None):
        self.client = client
        self.model = model
        root = Path(__file__).resolve().parent
        self.prompt_path = Path(prompt_path) if prompt_path else root / "prompts" / "highlevel_prompt.yaml"
        self.last_summary = ""
        self.last_plan_metadata: dict[str, object] = {}

    def load_prompt(self) -> dict[str, str]:
        if not self.prompt_path.exists():
            return {
                "system_prompt": "你是一个专业的机器人任务规划助手。输出必须是有效的JSON格式。",
                "prompt": "你是一个机器人任务规划助手。请将用户的复杂指令分解为简单的子任务。用户输入：{user_input}",
            }
        data = yaml.safe_load(self.prompt_path.read_text(encoding="utf-8")) or {}
        return {
            "system_prompt": str(data.get("system_prompt", "")).strip(),
            "prompt": str(data.get("prompt", "")).strip(),
        }

    def plan_tasks(
        self,
        user_input: str,
        planning_prompt: str,
        visual_context: str | None = None,
        scene_facts: dict | None = None,
        object_facts: dict | None = None,
    ) -> list[dict]:
        print("\n" + "=" * 60 + "\n🧠 [上层LLM] 任务规划中...\n" + "=" * 60)
        prompts = self.load_prompt()
        try:
            user_content = planning_prompt.format(
                user_input=user_input,
                visual_context=visual_context if visual_context else "无",
                scene_facts=json.dumps(scene_facts or {}, ensure_ascii=False),
                object_facts=json.dumps(object_facts or {}, ensure_ascii=False),
            )

            completion = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": prompts["system_prompt"]},
                    {"role": "user", "content": user_content},
                ],
                temperature=0.3,
                extra_body={"enable_thinking": False},
            )
            response_text = completion.choices[0].message.content.strip()
            if response_text.startswith("```"):
                response_text = response_text.split("```")[1]
                if response_text.startswith("json"):
                    response_text = response_text[4:]

            plan = json.loads(response_text)
            tasks, plan_meta = self.parse_plan_response(plan)
            summary = str(plan_meta.get("summary", ""))
            tasks, summary, override_meta = self._apply_scene_rule_overrides(tasks, summary, scene_facts, object_facts)
            if override_meta:
                plan_meta.update(override_meta)
            if not tasks:
                tasks, summary = self._build_navigation_fallback(
                    user_input,
                    visual_context,
                    scene_facts=scene_facts,
                    object_facts=object_facts,
                )
                if tasks:
                    print("⚠️  [规划兜底] LLM 返回空任务，已根据视觉上下文生成导航步骤")
                plan_meta = {
                    "scene_assessment": "",
                    "candidate_plans": [],
                    "selected_plan_id": "fallback_plan",
                    "summary": summary,
                }
            self.last_summary = summary
            self.last_plan_metadata = plan_meta
            print(f"✅ [规划完成] 共分解为 {len(tasks)} 个子任务\n📋 [任务概述] {summary}\n\n子任务序列：")
            for task in tasks:
                print(f"  步骤 {task['step']}: {task['task']} ({task['type']})")
                print(f"    建议函数: {task['function']}")
                print(f"    规划依据: {task['reason']}")
            return tasks
        except Exception as error:
            fallback_tasks, fallback_summary = self._build_navigation_fallback(
                user_input,
                visual_context,
                scene_facts=scene_facts,
                object_facts=object_facts,
            )
            if fallback_tasks:
                self.last_summary = fallback_summary
                self.last_plan_metadata = {
                    "scene_assessment": "",
                    "candidate_plans": [],
                    "selected_plan_id": "fallback_plan",
                    "summary": fallback_summary,
                }
                print(f"⚠️  [规划失败] {error}\n[规划兜底] 已根据视觉上下文生成导航步骤")
                print(f"📋 [任务概述] {fallback_summary}\n\n子任务序列：")
                for task in fallback_tasks:
                    print(f"  步骤 {task['step']}: {task['task']} ({task['type']})")
                    print(f"    建议函数: {task['function']}")
                    print(f"    规划依据: {task['reason']}")
                return fallback_tasks

            print(f"❌ [规划失败] {error}\n[回退] 将作为单个任务处理")
            self.last_summary = "规划失败，按单个任务处理"
            self.last_plan_metadata = {
                "scene_assessment": "",
                "candidate_plans": [],
                "selected_plan_id": "default_plan",
                "summary": self.last_summary,
            }
            return [self._normalize_task({"step": 1, "task": user_input, "type": "综合"}, 1)]

    def parse_plan_response(self, payload: dict) -> tuple[list[dict], dict[str, object]]:
        raw_tasks = payload.get("tasks", [])
        tasks = [self._normalize_task(task, idx) for idx, task in enumerate(raw_tasks, 1)]
        meta = {
            "scene_assessment": str(payload.get("scene_assessment", "")).strip(),
            "candidate_plans": payload.get("candidate_plans", []) or [],
            "selected_plan_id": str(payload.get("selected_plan_id", "default_plan")).strip() or "default_plan",
            "summary": str(payload.get("summary", "")).strip(),
        }
        return tasks, meta

    def _apply_scene_rule_overrides(
        self,
        tasks: list[dict],
        summary: str,
        scene_facts: dict | None,
        object_facts: dict | None,
    ) -> tuple[list[dict], str, dict[str, object] | None]:
        box_assisted = self._detect_box_assisted_geometry(scene_facts, object_facts)
        if box_assisted:
            has_box_plan = any(task.get("function") in {"push_box", "climb"} for task in tasks)
            if not has_box_plan:
                override_tasks, override_summary = self._build_box_assisted_plan(box_assisted)
                override_meta = {
                    "selected_plan_id": "rule_override_box_assist",
                    "summary": override_summary,
                }
                print("⚠️  [规则修正] 检测到箱子辅助攀爬场景，已将 walk 规划修正为 push_box + climb")
                return override_tasks, override_summary, override_meta

        single_climb = self._detect_single_climb_side(scene_facts)
        if not single_climb:
            return tasks, summary, None

        climb_side, climb_height = single_climb
        has_climb = any(task.get("function") == "climb" for task in tasks)
        if has_climb:
            return tasks, summary, None

        override_tasks, override_summary = self._build_single_climb_plan(climb_side, climb_height)
        override_meta = {
            "selected_plan_id": "rule_override_single_climb",
            "summary": override_summary,
        }
        print("⚠️  [规则修正] 检测到仅单侧可攀爬场景，已将 walk 规划修正为 climb")
        return override_tasks, override_summary, override_meta

    @staticmethod
    def _normalize_task(task: dict, default_step: int) -> dict:
        """补齐规划字段，保证执行层可以稳定读取。"""
        return {
            "step": task.get("step", default_step),
            "task": task.get("task", ""),
            "type": task.get("type", "未分类"),
            "function": task.get("function", "待LLM决定"),
            "reason": task.get("reason", "未提供规划依据"),
        }

    def _build_navigation_fallback(
        self,
        user_input: str,
        visual_context: str | None,
        scene_facts: dict | None = None,
        object_facts: dict | None = None,
    ) -> tuple[list[dict], str]:
        """当上层 LLM 返回空任务或异常时，基于规则生成导航步骤。"""
        if not self._is_navigation_request(user_input):
            return [], "无效输入，无法分解任务"

        context = visual_context or ""

        box_assisted = self._detect_box_assisted_geometry(scene_facts, object_facts)
        if box_assisted:
            return self._build_box_assisted_plan(box_assisted)

        single_climb = self._detect_single_climb_side(scene_facts)
        if single_climb:
            return self._build_single_climb_plan(*single_climb)

        if self._is_box_assisted_scene(context):
            box_side = self._detect_box_side(context)
            side_label = "左侧" if box_side == "left" else "右侧"
            return (
                [
                    self._normalize_task(
                        {
                            "step": 1,
                            "task": f"根据视觉信息，{side_label}存在可利用箱子。先播报环境并选择{side_label}路线",
                            "type": "路线选择",
                            "function": "way_select",
                            "reason": "机器人从中间出发，需要先切换到箱子所在路线",
                        },
                        1,
                    ),
                    self._normalize_task(
                        {
                            "step": 2,
                            "task": "将箱子推到高台旁边",
                            "type": "推箱子",
                            "function": "push_box",
                            "reason": "高台超过直接攀爬能力，需要先创造中间支撑点",
                        },
                        2,
                    ),
                    self._normalize_task(
                        {
                            "step": 3,
                            "task": "先攀爬到箱子顶部",
                            "type": "攀爬",
                            "function": "climb",
                            "reason": "先到达中间支撑点，为二段攀爬做准备",
                        },
                        3,
                    ),
                    self._normalize_task(
                        {
                            "step": 4,
                            "task": "从箱子顶部继续攀爬到高台",
                            "type": "攀爬",
                            "function": "climb",
                            "reason": "利用箱子降低实际攀爬高度到可执行范围内",
                        },
                        4,
                    ),
                    self._normalize_task(
                        {
                            "step": 5,
                            "task": "在高台上继续行走到前方目标点",
                            "type": "行走",
                            "function": "walk",
                            "reason": "完成翻越后，继续直行到目标点",
                        },
                        5,
                    ),
                ],
                "根据视觉上下文判断需要先选择箱子所在路线，再执行推箱子和攀爬动作到达目标点",
            )

        right_obstacle = self._has_side_obstacle(context, "right")
        left_obstacle = self._has_side_obstacle(context, "left")

        if right_obstacle and not left_obstacle:
            return (
                [
                    self._normalize_task(
                        {
                            "step": 1,
                            "task": "右侧存在障碍，先播报环境并选择左侧路线",
                            "type": "路线选择",
                            "function": "way_select",
                            "reason": "右侧路径受阻，左侧更开阔，选择左侧更简单更快",
                        },
                        1,
                    ),
                    self._normalize_task(
                        {
                            "step": 2,
                            "task": "沿左侧路线行走到前方目标点",
                            "type": "行走",
                            "function": "walk",
                            "reason": "完成路线切换后，沿左侧继续前进即可到达目标点",
                        },
                        2,
                    ),
                ],
                "基于右侧障碍和左侧可通行条件，选择左侧路线前往目标点",
            )

        if left_obstacle and not right_obstacle:
            return (
                [
                    self._normalize_task(
                        {
                            "step": 1,
                            "task": "左侧存在障碍，先播报环境并选择右侧路线",
                            "type": "路线选择",
                            "function": "way_select",
                            "reason": "左侧路径受阻，右侧更开阔，选择右侧更简单更快",
                        },
                        1,
                    ),
                    self._normalize_task(
                        {
                            "step": 2,
                            "task": "沿右侧路线行走到前方目标点",
                            "type": "行走",
                            "function": "walk",
                            "reason": "完成路线切换后，沿右侧继续前进即可到达目标点",
                        },
                        2,
                    ),
                ],
                "基于左侧障碍和右侧可通行条件，选择右侧路线前往目标点",
            )

        return (
            [
                self._normalize_task(
                    {
                        "step": 1,
                        "task": "前方路径可通行，直接沿当前路线行走到目标点",
                        "type": "行走",
                        "function": "walk",
                        "reason": "视觉信息未显示必须换道或使用复杂技能，直接前进最快",
                    },
                    1,
                )
            ],
            "基于当前视觉上下文未发现必须换道的关键障碍，直接行走到目标点",
        )

    def _detect_box_assisted_geometry(
        self,
        scene_facts: dict | None,
        object_facts: dict | None,
    ) -> dict[str, object] | None:
        if not scene_facts or not object_facts:
            return None

        constraints = object_facts.get("constraints") or scene_facts.get("constraints") or {}
        climb_limit = float(constraints.get("max_climb_height_m", 0.3))

        objects = object_facts.get("objects") or []
        boxes = [
            obj for obj in objects
            if obj.get("movable") and str(obj.get("type", "")).lower() == "box"
        ]
        platforms = [
            obj for obj in objects
            if str(obj.get("type", "")).lower() == "platform"
        ]
        if not boxes or not platforms:
            return None

        support_box = min(boxes, key=lambda item: float((item.get("size") or [0.0, 0.0, 0.0])[2]))
        target_platform = max(platforms, key=lambda item: float((item.get("size") or [0.0, 0.0, 0.0])[2]))
        box_height = round(float((support_box.get("size") or [0.0, 0.0, 0.0])[2]), 3)
        platform_height = round(float((target_platform.get("size") or [0.0, 0.0, 0.0])[2]), 3)
        remaining_height = round(platform_height - box_height, 3)

        if not (0 < box_height <= climb_limit):
            return None
        if platform_height <= climb_limit:
            return None
        if not (0 < remaining_height <= climb_limit):
            return None

        center = support_box.get("center") or target_platform.get("center") or [0.0, 0.0, 0.0]
        side = "left" if float(center[1]) >= 0 else "right"
        return {
            "side": side,
            "box_id": str(support_box.get("id", "box")),
            "box_height": box_height,
            "platform_id": str(target_platform.get("id", "platform")),
            "platform_height": platform_height,
            "remaining_height": remaining_height,
        }

    def _detect_single_climb_side(self, scene_facts: dict | None) -> tuple[str, float] | None:
        if not scene_facts:
            return None

        interactive_objects = scene_facts.get("interactive_objects") or []
        if interactive_objects:
            return None

        constraints = scene_facts.get("constraints") or {}
        climb_limit = float(constraints.get("max_climb_height_m", 0.3))
        terrain_features = scene_facts.get("terrain_features") or []
        features_by_side = {
            feature.get("side"): feature
            for feature in terrain_features
            if feature.get("side") in {"left", "right"}
        }
        if len(features_by_side) < 2:
            return None

        climbable_side = None
        blocked_other_side = False
        for side, feature in features_by_side.items():
            height = float(feature.get("height_m") or 0.0)
            feature_type = str(feature.get("type", ""))
            traversable = bool(feature.get("traversable"))
            if feature_type == "platform" and 0 < height <= climb_limit and traversable:
                climbable_side = (side, round(height, 3))
            elif height > climb_limit or not traversable:
                blocked_other_side = True

        if climbable_side and blocked_other_side:
            return climbable_side
        return None

    def _build_single_climb_plan(self, climb_side: str, climb_height: float) -> tuple[list[dict], str]:
        side_label = "左侧" if climb_side == "left" else "右侧"
        height_label = f"{climb_height:.1f}".rstrip("0").rstrip(".")
        return (
            [
                self._normalize_task(
                    {
                        "step": 1,
                        "task": f"{side_label}高台约{height_label}米且在单步攀爬上限内。先播报环境并选择{side_label}路线",
                        "type": "路线选择",
                        "function": "way_select",
                        "reason": f"{side_label}是唯一可通过攀爬到达的路线，需先切换到{side_label}入口",
                    },
                    1,
                ),
                self._normalize_task(
                    {
                        "step": 2,
                        "task": f"攀爬约{height_label}米到{side_label}高台",
                        "type": "攀爬",
                        "function": "climb",
                        "reason": f"{side_label}高台高度约{height_label}米，满足单步攀爬上限，应使用 climb 而不是直接 walk",
                    },
                    2,
                ),
            ],
            f"根据结构化场景判断，仅{side_label}约{height_label}米高台可攀爬，先选择{side_label}路线再执行攀爬",
        )

    def _is_navigation_request(self, user_input: str) -> bool:
        return any(keyword in user_input for keyword in self._NAVIGATION_KEYWORDS)

    def _build_box_assisted_plan(self, box_plan: dict[str, object]) -> tuple[list[dict], str]:
        side = str(box_plan["side"])
        side_label = "左侧" if side == "left" else "右侧"
        box_id = str(box_plan["box_id"])
        box_height = float(box_plan["box_height"])
        platform_id = str(box_plan["platform_id"])
        remaining_height = float(box_plan["remaining_height"])
        box_height_label = f"{box_height:.1f}".rstrip("0").rstrip(".")
        remaining_height_label = f"{remaining_height:.1f}".rstrip("0").rstrip(".")

        return (
            [
                self._normalize_task(
                    {
                        "step": 1,
                        "task": f"{side_label}存在可利用箱子 {box_id}。先播报环境并选择{side_label}路线",
                        "type": "路线选择",
                        "function": "way_select",
                        "reason": f"机器人需要先切换到{side_label}箱子所在路线，后续才能执行推箱子和攀爬",
                    },
                    1,
                ),
                self._normalize_task(
                    {
                        "step": 2,
                        "task": f"推箱子 {box_id} 到高台 {platform_id} 旁边",
                        "type": "推箱子",
                        "function": "push_box",
                        "reason": "高台超过单步攀爬上限，需要先把箱子移动到高台边形成中间支撑",
                    },
                    2,
                ),
                self._normalize_task(
                    {
                        "step": 3,
                        "task": f"先攀爬约{box_height_label}米到箱子 {box_id} 顶部",
                        "type": "攀爬",
                        "function": "climb",
                        "reason": f"先到达箱子顶部，利用 {box_id} 作为中间支撑点",
                    },
                    3,
                ),
                self._normalize_task(
                    {
                        "step": 4,
                        "task": f"再攀爬约{remaining_height_label}米到高台 {platform_id}",
                        "type": "攀爬",
                        "function": "climb",
                        "reason": f"利用箱子后，剩余相对高度约{remaining_height_label}米，满足单步攀爬上限",
                    },
                    4,
                ),
                self._normalize_task(
                    {
                        "step": 5,
                        "task": "在高台上继续行走到前方目标点",
                        "type": "行走",
                        "function": "walk",
                        "reason": "完成翻越后继续前进到目标点",
                    },
                    5,
                ),
            ],
            f"根据结构化物体几何信息，需先选择{side_label}路线，推动箱子并分两段攀爬后到达目标点",
        )

    def _has_side_obstacle(self, context: str, side: str) -> bool:
        if not context:
            return False
        side_prefix = "左" if side == "left" else "右"
        clear_pattern = rf"{side_prefix}(侧|边)?[^。；\n]*?(无障碍|无明显障碍|畅通|可通行|更开阔)"
        if re.search(clear_pattern, context):
            return False

        obstacle_words = "|".join(self._OBSTACLE_KEYWORDS)
        obstacle_pattern = rf"{side_prefix}(侧|边)?[^。；\n]*?({obstacle_words})"
        return re.search(obstacle_pattern, context) is not None

    def _is_box_assisted_scene(self, context: str) -> bool:
        if not context:
            return False
        if "箱子" not in context:
            return False
        if "高台" not in context and "平台" not in context:
            return False
        if "超过最大攀爬高度" in context:
            return True

        heights = [float(value) for value in re.findall(r"(\d+(?:\.\d+)?)\s*米", context)]
        return any(height > 0.3 for height in heights)

    @staticmethod
    def _detect_box_side(context: str) -> str:
        if re.search(r"右(侧|边)?[^。；\n]*?箱子", context):
            return "right"
        return "left"
