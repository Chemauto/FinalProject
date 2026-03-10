#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ASR 核心：录音并转成文字。"""

from __future__ import annotations

import base64
import subprocess
import sys
from pathlib import Path

from openai import OpenAI


class ASRCore:
    """语音识别核心。"""

    def __init__(
        self,
        api_key: str,
        base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1",
        model: str = "qwen3-asr-flash",
        record_seconds: int = 5,
    ):
        self.client = OpenAI(api_key=api_key, base_url=base_url)
        self.model = model
        self.record_seconds = record_seconds
        self.audio_dir = Path(__file__).resolve().parent / "audio_cache"
        self.audio_dir.mkdir(exist_ok=True)

    def record_audio(self, output_path: str | None = None) -> Path:
        """录制一段 wav 音频。"""
        audio_path = Path(output_path) if output_path else self.audio_dir / "input.wav"
        print(f"[ASR] 开始录音，时长 {self.record_seconds} 秒...", file=sys.stderr)
        subprocess.run(
            [
                "arecord",
                "-q",
                "-d", str(self.record_seconds),
                "-f", "S16_LE",
                "-r", "16000",
                "-c", "1",
                str(audio_path),
            ],
            check=True,
        )
        print(f"[ASR] 录音完成: {audio_path}", file=sys.stderr)
        return audio_path

    def transcribe(self, audio_path: str | Path) -> str:
        """把 wav 音频转成文字。"""
        path = Path(audio_path)
        audio_data = base64.b64encode(path.read_bytes()).decode("utf-8")
        data_uri = f"data:audio/wav;base64,{audio_data}"
        completion = self.client.chat.completions.create(
            model=self.model,
            messages=[
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "input_audio",
                            "input_audio": {
                                "data": data_uri,
                            },
                        },
                    ],
                }
            ],
            extra_body={"asr_options": {"language": "zh", "enable_itn": True}},
        )
        content = completion.choices[0].message.content
        if isinstance(content, str):
            return content.strip()
        return "".join(block.get("text", "") for block in content or []).strip()

    def listen_and_transcribe(self) -> str:
        """录音并直接返回识别文字。"""
        audio_path = self.record_audio()
        text = self.transcribe(audio_path)
        print(f"[ASR] 识别结果: {text}", file=sys.stderr)
        return text
