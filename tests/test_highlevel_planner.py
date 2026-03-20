import tempfile
import types
import unittest
from pathlib import Path

from LLM_Module.llm_highlevel import HighLevelPlanner


class _FakeCompletionMessage:
    def __init__(self, content: str):
        self.content = content


class _FakeChoice:
    def __init__(self, content: str):
        self.message = _FakeCompletionMessage(content)


class _FakeCompletion:
    def __init__(self, content: str):
        self.choices = [_FakeChoice(content)]


class HighLevelPlannerTests(unittest.TestCase):
    def test_parse_legacy_plan_format(self):
        planner = HighLevelPlanner(client=None, model="test")
        payload = {
            "tasks": [{"task": "沿右侧前进", "function": "walk"}],
            "summary": "legacy",
        }

        tasks, meta = planner.parse_plan_response(payload)

        self.assertEqual(len(tasks), 1)
        self.assertEqual(tasks[0]["function"], "walk")
        self.assertEqual(meta["selected_plan_id"], "default_plan")
        self.assertEqual(meta["summary"], "legacy")

    def test_parse_candidate_plan_format(self):
        planner = HighLevelPlanner(client=None, model="test")
        payload = {
            "scene_assessment": "右侧更简单",
            "candidate_plans": [
                {
                    "plan_id": "plan_a",
                    "strategy": "先右移再前进",
                    "tasks": [{"task": "先右移", "function": "way_select"}],
                }
            ],
            "selected_plan_id": "plan_a",
            "tasks": [{"task": "先右移", "function": "way_select"}],
            "summary": "new",
        }

        tasks, meta = planner.parse_plan_response(payload)

        self.assertEqual(meta["selected_plan_id"], "plan_a")
        self.assertEqual(meta["scene_assessment"], "右侧更简单")
        self.assertEqual(len(meta["candidate_plans"]), 1)
        self.assertEqual(tasks[0]["function"], "way_select")

    def test_build_navigation_fallback_prefers_single_climb_side(self):
        planner = HighLevelPlanner(client=None, model="test")
        scene_facts = {
            "terrain_features": [
                {"side": "left", "type": "platform", "height_m": 0.2, "traversable": True, "description": "左侧0.2米高台"},
                {"side": "right", "type": "platform", "height_m": 0.4, "traversable": False, "description": "右侧0.4米高台"},
            ],
            "interactive_objects": [],
            "route_options": [
                {"direction": "left", "status": "clear", "reason": "左侧0.2米高台，可攀爬"},
                {"direction": "right", "status": "blocked", "reason": "右侧0.4米高台，超过上限"},
            ],
            "constraints": {"max_climb_height_m": 0.3},
        }

        tasks, summary = planner._build_navigation_fallback(
            "前往目标点",
            "左侧0.2米高台，右侧0.4米高台，没有箱子",
            scene_facts=scene_facts,
        )

        self.assertEqual([task["function"] for task in tasks], ["way_select", "climb"])
        self.assertIn("左侧", tasks[0]["task"])
        self.assertIn("0.2米", tasks[1]["task"])
        self.assertIn("攀爬", summary)

    def test_plan_tasks_overrides_walk_only_plan_in_single_climb_scene(self):
        scene_facts = {
            "terrain_features": [
                {"side": "left", "type": "platform", "height_m": 0.2, "traversable": True, "description": "左侧0.2米高台"},
                {"side": "right", "type": "platform", "height_m": 0.4, "traversable": False, "description": "右侧0.4米高台"},
            ],
            "interactive_objects": [],
            "route_options": [
                {"direction": "left", "status": "clear", "reason": "左侧0.2米高台，可攀爬"},
                {"direction": "right", "status": "blocked", "reason": "右侧0.4米高台，超过上限"},
            ],
            "constraints": {"max_climb_height_m": 0.3},
        }

        def create_completion(*, model, messages, temperature, extra_body):
            return _FakeCompletion(
                '{"tasks":['
                '{"step":1,"task":"先选择左侧路线","type":"路线选择","function":"way_select","reason":"左侧更近"},'
                '{"step":2,"task":"沿左侧路线直接行走到目标点","type":"行走","function":"walk","reason":"继续前进即可"}'
                '],"summary":"选择左侧路线后直接行走"}'
            )

        fake_client = types.SimpleNamespace(
            chat=types.SimpleNamespace(
                completions=types.SimpleNamespace(create=create_completion)
            )
        )

        with tempfile.TemporaryDirectory() as tmpdir:
            prompt_path = Path(tmpdir) / "prompt.yaml"
            prompt_path.write_text(
                "system_prompt: |\n"
                "  你是测试高层规划器。\n"
                "prompt: |\n"
                "  visual_context={visual_context}\n"
                "  scene_facts={scene_facts}\n"
                "  user_input={user_input}\n"
                "  replan_context={replan_context}\n",
                encoding="utf-8",
            )
            planner = HighLevelPlanner(client=fake_client, model="fake-model", prompt_path=str(prompt_path))
            tasks = planner.plan_tasks(
                "前往目标点",
                planner.load_prompt()["prompt"],
                visual_context="左侧0.2米高台，右侧0.4米高台，没有箱子",
                scene_facts=scene_facts,
            )

        self.assertEqual([task["function"] for task in tasks], ["way_select", "climb"])
        self.assertIn("0.2米", tasks[1]["task"])
        self.assertIn("攀爬", planner.last_summary)


if __name__ == "__main__":
    unittest.main()
