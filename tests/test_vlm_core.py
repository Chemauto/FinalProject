import json
import unittest
from pathlib import Path
from unittest.mock import patch

from VLM_Module.vlm_core import VLMCore


class _FakeResponse:
    def __init__(self, content: str = "mocked"):
        self.choices = [type("Choice", (), {"message": type("Message", (), {"content": content})()})()]


class _FakeCompletions:
    def __init__(self):
        self.last_kwargs = None
        self.response_content = "mocked"

    def create(self, **kwargs):
        self.last_kwargs = kwargs
        return _FakeResponse(self.response_content)


class _FakeChat:
    def __init__(self):
        self.completions = _FakeCompletions()


class _FakeClient:
    def __init__(self):
        self.chat = _FakeChat()


class TestVLMCore(unittest.TestCase):
    @patch("VLM_Module.vlm_core.OpenAI")
    def test_loaded_prompt_requires_structured_json_output(self, mock_openai):
        mock_openai.return_value = _FakeClient()
        core = VLMCore(api_key="test-key")

        prompts = core._load_prompts()

        self.assertIn("严格输出 JSON", prompts["system"])
        self.assertIn("suspected_height_diff", prompts["system"])
        self.assertIn("不要判断机器人能否通行", prompts["system"])
        self.assertNotIn("自然语言描述", prompts["user"])

    @patch("VLM_Module.vlm_core.OpenAI")
    def test_describe_structured_parses_fenced_json_and_normalizes_schema(self, mock_openai):
        fake_client = _FakeClient()
        fake_client.chat.completions.response_content = (
            """```json
            {"ground":"黑色网格地面","left_side":"左侧灰色墙体","right_side":"右侧粉色平台","front_area":"前方有平台分界","obstacles":"右侧粉色平台","suspected_height_diff":"true","uncertainties":"无法确认绝对高度"}
            ```"""
        )
        mock_openai.return_value = fake_client

        image_path = str(Path("VLM_Module/assets/3.png").resolve())
        core = VLMCore(api_key="test-key")
        result = core.describe_structured(image_path=image_path)

        self.assertEqual(result["ground"], "黑色网格地面")
        self.assertEqual(result["obstacles"], ["右侧粉色平台"])
        self.assertEqual(result["uncertainties"], ["无法确认绝对高度"])
        self.assertTrue(result["suspected_height_diff"])

    @patch("VLM_Module.vlm_core.OpenAI")
    def test_describe_returns_json_string_and_uses_low_default_temperature(self, mock_openai):
        fake_client = _FakeClient()
        fake_client.chat.completions.response_content = (
            '{"ground":"黑色网格地面","left_side":"左侧灰色墙体","right_side":"右侧粉色平台","front_area":"前方平台","obstacles":["右侧粉色平台"],"suspected_height_diff":true,"uncertainties":["绝对高度未知"]}'
        )
        mock_openai.return_value = fake_client

        image_path = str(Path("VLM_Module/assets/3.png").resolve())
        core = VLMCore(api_key="test-key")
        result = core.describe(image_path=image_path)

        parsed = json.loads(result)
        self.assertEqual(parsed["right_side"], "右侧粉色平台")
        self.assertEqual(fake_client.chat.completions.last_kwargs["temperature"], 0.1)


if __name__ == "__main__":
    unittest.main()
