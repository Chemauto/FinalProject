#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""语音交互入口：语音转文字后复用原有交互流程。"""

from __future__ import annotations

import os
import sys

from asr_core import ASRCore
from interactive import build_llm_agent, process_user_input, show_welcome


def main():
    api_key = os.getenv("Test_API_KEY")
    if not api_key:
        print("❌ 错误: 未设置 Test_API_KEY 环境变量", file=sys.stderr)
        sys.exit(1)

    llm_agent, tools = build_llm_agent()
    asr = ASRCore(api_key=api_key)
    show_welcome(
        llm_agent,
        tools,
        title="ASR Interactive Interface",
        input_hint="回车开始录音，输入 'quit' 或 'exit' 退出",
    )

    while True:
        try:
            command = input("\n🎤 按回车开始录音: ").strip()
            if command.lower() in ["quit", "exit", "q"]:
                print("👋 再见!", file=sys.stderr)
                break
            user_input = asr.listen_and_transcribe()
            if not process_user_input(user_input, llm_agent, tools):
                break
        except KeyboardInterrupt:
            print("\n\n👋 再见!", file=sys.stderr)
            break
        except Exception as error:
            print(f"\n❌ [ASR错误] {error}", file=sys.stderr)


if __name__ == "__main__":
    main()
