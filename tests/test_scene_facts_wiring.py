import tempfile
import types
import unittest
from pathlib import Path

from Interactive_Module.interactive import process_user_input
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


class SceneFactsWiringTests(unittest.TestCase):
    def test_highlevel_planner_accepts_scene_facts_and_replan_context(self):
        captured = {}

        def create_completion(*, model, messages, temperature, extra_body):
            captured["messages"] = messages
            return _FakeCompletion('{"tasks":[{"task":"沿右侧前进","function":"walk"}],"summary":"ok"}')

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
                "  scene_facts={scene_facts}\n"
                "  replan_context={replan_context}\n"
                "  visual_context={visual_context}\n"
                "  user_input={user_input}\n",
                encoding="utf-8",
            )

            planner = HighLevelPlanner(client=fake_client, model="fake-model", prompt_path=str(prompt_path))
            tasks = planner.plan_tasks(
                "前往前方目标点",
                planner.load_prompt()["prompt"],
                visual_context="右侧通行",
                scene_facts={"summary": "右侧更简单"},
                replan_context={"failure_feedback": {"signal": "FAILURE"}},
            )

        self.assertEqual(tasks[0]["function"], "walk")
        user_message = captured["messages"][1]["content"]
        self.assertIn('scene_facts={"summary": "右侧更简单"}', user_message)
        self.assertIn('replan_context={"failure_feedback": {"signal": "FAILURE"}}', user_message)

    def test_process_user_input_passes_scene_facts_to_run_pipeline(self):
        captured = {}

        class FakeAgent:
            def run_pipeline(self, user_input, tools, execute_tool_fn, visual_context=None, scene_facts=None):
                captured["user_input"] = user_input
                captured["visual_context"] = visual_context
                captured["scene_facts"] = scene_facts
                return [{"success": True}]

        class FakeVLM:
            def describe(self):
                return '{"left_side":"0.2米高台","right_side":"无障碍"}'

            def describe_structured(self):
                return {
                    "ground": "flat",
                    "left_side": "0.2米高台",
                    "right_side": "无障碍",
                    "front_area": "前方可通行",
                    "obstacles": [],
                    "suspected_height_diff": True,
                    "uncertainties": [],
                }

        should_continue = process_user_input(
            user_input="前往前方目标点",
            llm_agent=FakeAgent(),
            tools=[],
            vlm_core=FakeVLM(),
        )

        self.assertTrue(should_continue)
        self.assertEqual(captured["visual_context"], '{"left_side":"0.2米高台","right_side":"无障碍"}')
        self.assertIsInstance(captured["scene_facts"], dict)
        self.assertEqual(captured["scene_facts"]["constraints"]["max_climb_height_m"], 0.3)


if __name__ == "__main__":
    unittest.main()
