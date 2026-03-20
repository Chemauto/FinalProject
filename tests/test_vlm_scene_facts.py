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


if __name__ == "__main__":
    unittest.main()
