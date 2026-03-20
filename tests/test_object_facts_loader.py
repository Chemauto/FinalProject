import json
import tempfile
import unittest
from pathlib import Path

from LLM_Module.object_facts_loader import load_object_facts


class ObjectFactsLoaderTests(unittest.TestCase):
    def test_load_object_facts_normalizes_vectors_constraints_and_objects(self):
        payload = {
            "navigation_goal": [5, 0, 0],
            "robot_pose": [0, "0.5", 0.0],
            "constraints": {
                "max_climb_height_m": "0.3",
                "push_only_on_ground": True,
                "climb_requires_adjacency": False,
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

        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "object_facts.json"
            path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")

            facts = load_object_facts(path)

        self.assertEqual(facts["navigation_goal"], [5.0, 0.0, 0.0])
        self.assertEqual(facts["robot_pose"], [0.0, 0.5, 0.0])
        self.assertEqual(facts["constraints"]["max_climb_height_m"], 0.3)
        self.assertTrue(facts["constraints"]["push_only_on_ground"])
        self.assertFalse(facts["constraints"]["climb_requires_adjacency"])
        self.assertEqual(len(facts["objects"]), 2)
        self.assertEqual(facts["objects"][0]["center"], [2.3, 0.25, 0.0])
        self.assertEqual(facts["objects"][1]["size"], [0.6, 0.6, 0.4])

    def test_load_object_facts_returns_none_for_missing_file(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "missing.json"
            facts = load_object_facts(path)

        self.assertIsNone(facts)

    def test_load_object_facts_raises_for_invalid_vec3(self):
        payload = {
            "navigation_goal": [5, 0],
            "robot_pose": [0, 0, 0],
            "objects": [],
        }

        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "object_facts.json"
            path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")

            with self.assertRaises(ValueError):
                load_object_facts(path)


if __name__ == "__main__":
    unittest.main()
