import types
import unittest

from LLM_Module.llm_core import LLMAgent


class ReplanFlowTests(unittest.TestCase):
    def test_run_pipeline_replans_once_after_failure(self):
        agent = LLMAgent.__new__(LLMAgent)
        agent.highlevel = types.SimpleNamespace(last_summary="", last_plan_metadata={})
        agent.lowlevel = object()

        plan_calls = []
        plan_sequence = [
            [
                {
                    "step": 1,
                    "task": "推动箱子",
                    "type": "推箱子",
                    "function": "push_box",
                    "reason": "先推箱子再通过",
                }
            ],
            [
                {
                    "step": 1,
                    "task": "改走右侧路线",
                    "type": "路线选择",
                    "function": "way_select",
                    "reason": "箱子不可用，改走替代路线",
                }
            ],
        ]

        def fake_plan_tasks(self, user_input, tools, visual_context=None, scene_facts=None, replan_context=None):
            call_index = len(plan_calls)
            plan_calls.append({"scene_facts": scene_facts, "replan_context": replan_context})
            self.highlevel.last_summary = "初始计划" if call_index == 0 else "重规划计划"
            self.highlevel.last_plan_metadata = {
                "selected_plan_id": "plan_a" if call_index == 0 else "plan_b",
            }
            return plan_sequence[call_index]

        execution_results = [
            {
                "success": False,
                "action": "push_box",
                "task": "推动箱子",
                "feedback": {
                    "signal": "FAILURE",
                    "skill": "push_box",
                    "message": "箱子过重，推动失败",
                },
                "result": {},
            },
            {
                "success": True,
                "action": "way_select",
                "task": "改走右侧路线",
                "feedback": {
                    "signal": "SUCCESS",
                    "skill": "way_select",
                    "message": "已切换到右侧路线",
                },
                "result": {"result": {"direction": "right"}},
            },
        ]

        def fake_execute_single_task(self, task_info, tools, execute_tool_fn, previous_result=None, visual_context=None):
            return execution_results.pop(0)

        agent.plan_tasks = types.MethodType(fake_plan_tasks, agent)
        agent.execute_single_task = types.MethodType(fake_execute_single_task, agent)

        results = agent.run_pipeline(
            user_input="前往前方目标点",
            tools=[],
            execute_tool_fn=lambda *_args, **_kwargs: {},
            visual_context="左侧有箱子但推动失败",
        )

        self.assertEqual(len(plan_calls), 2)
        self.assertIsNone(plan_calls[0]["replan_context"])
        self.assertEqual(plan_calls[1]["replan_context"]["failure_feedback"]["signal"], "FAILURE")
        self.assertEqual(plan_calls[1]["replan_context"]["current_plan_id"], "plan_a")
        self.assertEqual(len(results), 2)
        self.assertEqual(results[-1]["action"], "way_select")

    def test_run_pipeline_does_not_replan_more_than_once(self):
        agent = LLMAgent.__new__(LLMAgent)
        agent.highlevel = types.SimpleNamespace(last_summary="", last_plan_metadata={})
        agent.lowlevel = object()

        plan_calls = []
        plan_sequence = [
            [{"step": 1, "task": "推箱子", "type": "推箱子", "function": "push_box", "reason": "初始方案"}],
            [{"step": 1, "task": "改走左侧", "type": "路线选择", "function": "way_select", "reason": "替代方案"}],
        ]

        def fake_plan_tasks(self, user_input, tools, visual_context=None, scene_facts=None, replan_context=None):
            call_index = len(plan_calls)
            plan_calls.append({"replan_context": replan_context})
            self.highlevel.last_summary = "计划"
            self.highlevel.last_plan_metadata = {"selected_plan_id": f"plan_{call_index}"}
            return plan_sequence[call_index]

        def fake_execute_single_task(self, task_info, tools, execute_tool_fn, previous_result=None, visual_context=None):
            return {
                "success": False,
                "action": task_info["function"],
                "task": task_info["task"],
                "feedback": {
                    "signal": "FAILURE",
                    "skill": task_info["function"],
                    "message": "执行失败",
                },
                "result": {},
            }

        agent.plan_tasks = types.MethodType(fake_plan_tasks, agent)
        agent.execute_single_task = types.MethodType(fake_execute_single_task, agent)

        results = agent.run_pipeline(
            user_input="前往前方目标点",
            tools=[],
            execute_tool_fn=lambda *_args, **_kwargs: {},
            visual_context="当前路径受阻",
        )

        self.assertEqual(len(plan_calls), 2)
        self.assertEqual(len(results), 2)
        self.assertFalse(results[-1]["success"])


if __name__ == "__main__":
    unittest.main()
