import unittest

from LLM_Module.parameter_calculator import ParameterCalculator


class ParameterCalculatorTests(unittest.TestCase):
    def test_annotate_tasks_builds_box_assisted_parameters_from_geometry(self):
        calculator = ParameterCalculator()
        tasks = [
            {"step": 1, "task": "先选择左侧路线", "type": "路线选择", "function": "way_select", "reason": "切换路线"},
            {"step": 2, "task": "将箱子推到高台旁边", "type": "推箱子", "function": "push_box", "reason": "搭建中间支撑"},
            {"step": 3, "task": "先攀爬到箱子顶部", "type": "攀爬", "function": "climb", "reason": "到达支撑点"},
            {"step": 4, "task": "从箱子顶部继续攀爬到高台", "type": "攀爬", "function": "climb", "reason": "到达高台"},
        ]
        object_facts = {
            "navigation_goal": [5.0, 0.0, 0.0],
            "robot_pose": [0.0, 0.0, 0.0],
            "constraints": {
                "max_climb_height_m": 0.3,
                "push_only_on_ground": True,
                "climb_requires_adjacency": True,
            },
            "objects": [
                {
                    "id": "box1",
                    "type": "box",
                    "center": [2.3, 0.25, 0.0],
                    "size": [0.8, 0.5, 0.2],
                    "movable": True,
                },
                {
                    "id": "platform_front",
                    "type": "platform",
                    "center": [3.5, 0.12, 0.0],
                    "size": [0.6, 0.6, 0.4],
                    "movable": False,
                },
            ],
        }

        annotated = calculator.annotate_tasks(tasks, object_facts)

        self.assertEqual(annotated[0]["calculated_parameters"]["direction"], "left")
        self.assertEqual(annotated[0]["parameter_context"]["route_side"], "left")

        self.assertEqual(annotated[1]["calculated_parameters"]["box_height"], 0.2)
        self.assertEqual(annotated[1]["parameter_context"]["target_position_xyz"], [2.8, 0.12, 0.0])

        self.assertEqual(annotated[2]["calculated_parameters"]["height"], 0.2)
        self.assertEqual(annotated[2]["calculated_parameters"]["stage"], "box1")
        self.assertEqual(annotated[3]["calculated_parameters"]["height"], 0.2)
        self.assertEqual(annotated[3]["calculated_parameters"]["stage"], "platform_front")

    def test_annotate_tasks_builds_single_climb_parameters(self):
        calculator = ParameterCalculator()
        tasks = [
            {"step": 1, "task": "先选择左侧路线", "type": "路线选择", "function": "way_select", "reason": "切换路线"},
            {"step": 2, "task": "攀爬到左侧高台", "type": "攀爬", "function": "climb", "reason": "唯一可行路线"},
        ]
        object_facts = {
            "navigation_goal": [5.0, 0.0, 0.0],
            "robot_pose": [0.0, 0.0, 0.0],
            "constraints": {
                "max_climb_height_m": 0.3,
                "push_only_on_ground": True,
                "climb_requires_adjacency": True,
            },
            "objects": [
                {
                    "id": "platform_left",
                    "type": "platform",
                    "center": [2.5, 0.3, 0.0],
                    "size": [0.8, 0.8, 0.2],
                    "movable": False,
                }
            ],
        }

        annotated = calculator.annotate_tasks(tasks, object_facts)

        self.assertEqual(annotated[0]["calculated_parameters"]["direction"], "left")
        self.assertEqual(annotated[1]["calculated_parameters"]["height"], 0.2)
        self.assertEqual(annotated[1]["calculated_parameters"]["stage"], "platform_left")


if __name__ == "__main__":
    unittest.main()
