import unittest

from VLM_Module.vlm_core import VLMCore


class SceneFactsTests(unittest.TestCase):
    def test_build_scene_facts_includes_constraints_and_route_options(self):
        payload = {
            "ground": "flat",
            "left_side": "0.2米高台",
            "right_side": "无障碍",
            "front_area": "前方可通行",
            "obstacles": ["左侧高台"],
            "suspected_height_diff": True,
            "uncertainties": [],
        }

        facts = VLMCore.build_scene_facts(payload)

        self.assertEqual(facts["constraints"]["max_climb_height_m"], 0.3)
        self.assertEqual(facts["summary"], "左侧存在约0.2米高台，右侧可通行")
        self.assertTrue(any(option["direction"] == "right" for option in facts["route_options"]))
        self.assertTrue(any(feature["side"] == "left" for feature in facts["terrain_features"]))

    def test_build_scene_facts_from_object_facts_extracts_geometry(self):
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

        facts = VLMCore.build_scene_facts_from_object_facts(object_facts)

        self.assertEqual(facts["constraints"]["max_climb_height_m"], 0.3)
        self.assertEqual(facts["interactive_objects"][0]["name"], "box1")
        self.assertEqual(facts["interactive_objects"][0]["height_m"], 0.2)
        self.assertTrue(any(feature["height_m"] == 0.4 for feature in facts["terrain_features"]))
        self.assertEqual(facts["route_options"][0]["direction"], "left")

    def test_merge_scene_facts_prefers_object_facts_and_keeps_uncertainties(self):
        vlm_facts = {
            "summary": "右侧看起来更开阔",
            "terrain_features": [
                {"side": "right", "type": "path", "height_m": 0.0, "traversable": True, "description": "右侧开阔"}
            ],
            "interactive_objects": [],
            "route_options": [{"direction": "right", "status": "clear", "reason": "右侧开阔"}],
            "constraints": {"max_climb_height_m": 0.3},
            "uncertainties": ["图像里平台边缘不清晰"],
        }
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

        facts = VLMCore.merge_scene_facts(vlm_facts, object_facts)

        self.assertEqual(facts["summary"], "左侧存在约0.4米高台，左侧有可推动箱子 box1")
        self.assertEqual(facts["terrain_features"][0]["height_m"], 0.4)
        self.assertEqual(facts["interactive_objects"][0]["name"], "box1")
        self.assertIn("图像里平台边缘不清晰", facts["uncertainties"])


if __name__ == "__main__":
    unittest.main()
