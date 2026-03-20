import types
import unittest

from LLM_Module.llm_core import LLMAgent
from LLM_Module.llm_lowlevel import LowLevelExecutor


class LowLevelExecutionTests(unittest.TestCase):
    def test_execute_single_task_uses_calculated_parameters_without_llm(self):
        def should_not_call_llm(*args, **kwargs):
            raise AssertionError("LLM should not be called when calculated_parameters are available")

        fake_client = types.SimpleNamespace(
            chat=types.SimpleNamespace(
                completions=types.SimpleNamespace(create=should_not_call_llm)
            )
        )
        captured = {}

        def execute_tool(function_name, function_args):
            captured["function_name"] = function_name
            captured["function_args"] = function_args
            return {
                "success": True,
                "result": {"result": {"target_position": function_args["target_position"]}},
                "feedback": {"signal": "SUCCESS", "skill": function_name, "message": "ok"},
            }

        executor = LowLevelExecutor(fake_client, "fake-model")
        result = executor.execute_single_task(
            {
                "task": "推箱子 box1 到高台旁边",
                "type": "推箱子",
                "function": "push_box",
                "reason": "搭建中间支撑",
                "calculated_parameters": {
                    "box_height": 0.2,
                    "target_position": "[2.8, 0.12, 0.0]",
                },
                "parameter_context": {
                    "support_object": "box1",
                    "target_object": "platform_front",
                    "target_position_xyz": [2.8, 0.12, 0.0],
                },
            },
            tools=[],
            execute_tool_fn=execute_tool,
        )

        self.assertTrue(result["success"])
        self.assertEqual(captured["function_name"], "push_box")
        self.assertEqual(captured["function_args"]["box_height"], 0.2)
        self.assertEqual(captured["function_args"]["target_position"], "[2.8, 0.12, 0.0]")


class PipelineParameterInjectionTests(unittest.TestCase):
    def test_run_pipeline_injects_calculated_parameters_before_execution(self):
        agent = LLMAgent.__new__(LLMAgent)
        agent.highlevel = types.SimpleNamespace(last_summary="", last_plan_metadata={})
        agent.lowlevel = object()

        captured = {}

        class FakeCalculator:
            def annotate_tasks(self, tasks, object_facts):
                captured["object_facts"] = object_facts
                annotated = [dict(tasks[0])]
                annotated[0]["calculated_parameters"] = {"height": 0.2, "stage": "platform_left"}
                return annotated

        def fake_plan_tasks(self, user_input, tools, visual_context=None, scene_facts=None, object_facts=None, replan_context=None):
            self.highlevel.last_summary = "单侧攀爬"
            self.highlevel.last_plan_metadata = {"selected_plan_id": "plan_a"}
            return [
                {
                    "step": 1,
                    "task": "攀爬到左侧高台",
                    "type": "攀爬",
                    "function": "climb",
                    "reason": "唯一可行路线",
                }
            ]

        def fake_execute_single_task(self, task_info, tools, execute_tool_fn, previous_result=None, visual_context=None):
            captured["task_info"] = task_info
            return {
                "success": True,
                "action": task_info["function"],
                "task": task_info["task"],
                "feedback": {"signal": "SUCCESS", "skill": task_info["function"], "message": "ok"},
                "result": {"result": {"height": 0.2}},
            }

        agent.parameter_calculator = FakeCalculator()
        agent.plan_tasks = types.MethodType(fake_plan_tasks, agent)
        agent.execute_single_task = types.MethodType(fake_execute_single_task, agent)

        results = agent.run_pipeline(
            user_input="前往目标点",
            tools=[],
            execute_tool_fn=lambda *_args, **_kwargs: {},
            visual_context="左侧0.2米高台",
            object_facts={
                "navigation_goal": [5.0, 0.0, 0.0],
                "robot_pose": [0.0, 0.0, 0.0],
                "constraints": {"max_climb_height_m": 0.3},
                "objects": [],
            },
        )

        self.assertEqual(len(results), 1)
        self.assertEqual(captured["object_facts"]["navigation_goal"], [5.0, 0.0, 0.0])
        self.assertEqual(captured["task_info"]["calculated_parameters"]["height"], 0.2)


if __name__ == "__main__":
    unittest.main()
