#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LLM Core - LLM模块核心入口（适配层）

提供向后兼容的接口，内部使用新的模块化架构：
- HighLevelLLM: 任务规划
- LowLevelLLM: 执行控制
- VLMCore: 视觉理解（可选）
- AdaptiveController: 自适应控制（可选）
"""
import os
from typing import Callable, List, Dict, Any

# 导入新的模块化架构
from .high_level_llm import HighLevelLLM
from .low_level_llm import LowLevelLLM, ExecutionStatus
from .adaptive_controller import AdaptiveController
from .vlm_core import VLMCore


class LLMAgent:
    """
    LLM代理（主入口）

    整合所有 LLM 相关功能，提供简单的接口供外部调用。

    功能：
    1. 任务规划（High-Level LLM + VLM）
    2. 任务执行（Low-Level LLM）
    3. 自适应控制（可选）
    """

    def __init__(self,
                 api_key: str,
                 base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1",
                 model: str = "qwen3-32b",
                 prompt_path: str = None,
                 enable_vlm: bool = True,
                 vlm_prompt_path: str = None,
                 enable_adaptive: bool = False):
        """
        初始化LLM代理

        Args:
            api_key: API密钥
            base_url: API基础URL
            model: 文本LLM模型名称
            prompt_path: 规划提示词文件路径
            enable_vlm: 是否启用VLM环境理解（默认True）
            vlm_prompt_path: VLM提示词文件路径（可选）
            enable_adaptive: 是否启用自适应控制（默认False）
        """
        self.api_key = api_key
        self.base_url = base_url
        self.enable_vlm = enable_vlm
        self.enable_adaptive = enable_adaptive

        # ==================== 初始化 VLM ====================
        self.vlm_core = None
        if enable_vlm:
            # 如果未指定 vlm_prompt_path，自动查找
            if not vlm_prompt_path and prompt_path:
                prompts_dir = os.path.dirname(prompt_path)
                vlm_prompt_path = os.path.join(prompts_dir, "vlm_perception.yaml")

            self.vlm_core = VLMCore(
                vlm_prompt_path=vlm_prompt_path,
                use_ollama=True,  # 默认使用本地 Ollama
                ollama_model="qwen3-vl:4b"
            )
        # ===================================================

        # ==================== 初始化 High-Level LLM ====================
        self.high_level_llm = HighLevelLLM(
            api_key=api_key,
            base_url=base_url,
            model=model,
            prompt_path=prompt_path,
            vlm_core=self.vlm_core  # 传入 VLM 实例
        )
        # ================================================================

        # ==================== 初始化 Low-Level LLM ====================
        self.low_level_llm = LowLevelLLM(
            api_key=api_key,
            base_url=base_url,
            model=model
        )
        # ================================================================

        # ==================== 初始化自适应控制器 ====================
        self.adaptive_controller = None
        if enable_adaptive:
            self.adaptive_controller = AdaptiveController(
                high_level_llm=self.high_level_llm,
                low_level_llm=self.low_level_llm
            )
        # ================================================================

        # 兼容旧代码
        self.prompt_path = prompt_path
        self._planning_prompt_template = None

    @property
    def client(self):
        """获取OpenAI客户端（转发到high_level_llm）"""
        return self.high_level_llm.client

    @property
    def model(self):
        """获取模型名称（转发到high_level_llm）"""
        return self.high_level_llm.model

    @property
    def planning_prompt_template(self):
        """获取规划提示词模板"""
        return self.high_level_llm.prompt_template

    @planning_prompt_template.setter
    def planning_prompt_template(self, value):
        """设置规划提示词模板（会更新到high_level_llm）"""
        self.high_level_llm.prompt_template = value
        self._planning_prompt_template = value

    def plan_tasks(self, user_input: str, tools: List[Dict], image_path: str = None) -> List[Dict]:
        """
        上层LLM：将用户输入分解为子任务序列

        Args:
            user_input: 用户输入
            tools: 工具列表
            image_path: 环境图像路径（可选，用于VLM理解）

        Returns:
            任务列表
        """
        # 提取技能名称
        available_skills = [tool.get("function", {}).get("name", "unknown") for tool in tools]

        # 调用 HighLevelLLM（传递 image_path）
        return self.high_level_llm.plan_tasks(
            user_input=user_input,
            available_skills=available_skills,
            image_path=image_path
        )

    def execute_single_task(self,
                            task_description: str,
                            tools: List[Dict],
                            execute_tool_fn: Callable,
                            previous_result: Any = None) -> Dict:
        """
        下层LLM：执行单个子任务

        Args:
            task_description: 任务描述
            tools: 可用工具列表
            execute_tool_fn: 工具执行函数
            previous_result: 上一步的执行结果

        Returns:
            执行结果
        """
        # 调用 LowLevelLLM
        result = self.low_level_llm.execute_task(
            task_description=task_description,
            tools=tools,
            execute_tool_fn=execute_tool_fn,
            previous_result=previous_result
        )

        # 转换返回格式以兼容旧代码
        if result.get("status") == ExecutionStatus.SUCCESS.value:
            return {
                "success": True,
                "action": result.get("action"),
                "task": result.get("task"),
                "result": result.get("result")
            }
        else:
            return {
                "success": False,
                "error": result.get("error"),
                "task": result.get("task")
            }

    def run_pipeline(self,
                     user_input: str,
                     tools: List[Dict],
                     execute_tool_fn: Callable,
                     image_path: str = None,
                     env_state_provider: Callable[[], Dict[str, Any]] = None) -> List[Dict]:
        """
        运行完整的双层LLM流程

        新功能：如果 enable_adaptive=True，将使用自适应控制器

        Args:
            user_input: 用户输入
            tools: 可用工具列表
            execute_tool_fn: 工具执行函数
            image_path: 环境图像路径（可选，用于VLM理解）

        Returns:
            执行结果列表
        """
        print("\n" + "█"*60)
        print(f"📥 [用户输入] {user_input}")
        print("█"*60)

        # 如果启用自适应控制，使用 AdaptiveController
        if self.enable_adaptive and self.adaptive_controller:
            import asyncio

            # 提取技能名称
            available_skills = [tool.get("function", {}).get("name", "unknown") for tool in tools]

            input1 = {"text": user_input, "lang": "zh", "source": "console"}
            input3 = {
                "skills": [
                    {
                        "name": tool.get("function", {}).get("name", "unknown"),
                        "args_schema": tool.get("function", {}).get("parameters", {}).get("properties", {}),
                        "description": tool.get("function", {}).get("description", "")
                    }
                    for tool in tools
                ]
            }
            input2 = {}
            if env_state_provider:
                try:
                    snapshot = env_state_provider() or {}
                    input2 = {
                        "sensor_frame": snapshot,
                        "environment_version": int(snapshot.get("environment_version", 0) or 0),
                        "vlm_summary": ""
                    }
                except Exception:
                    input2 = {}

            # 运行异步控制器
            try:
                loop = asyncio.get_event_loop()
            except RuntimeError:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)

            results = loop.run_until_complete(
                self.adaptive_controller.run(
                    user_input=user_input,
                    tools=tools,
                    execute_tool_fn=execute_tool_fn,
                    available_skills=available_skills,
                    env_state=None,
                    env_state_provider=env_state_provider,
                    input1=input1,
                    input2=input2,
                    input3=input3,
                    image_path=image_path,
                )
            )

            return results

        # 否则，使用旧的同步流程（向后兼容）
        try:
            tasks = self.plan_tasks(user_input, tools, image_path)

            if not tasks:
                return []

            print("\n" + "█"*60)
            print("🚀 [开始执行] 按顺序执行子任务")
            print("█"*60)

            results = []
            previous_result = None

            for idx, task in enumerate(tasks, 1):
                print(f"\n【步骤 {idx}/{len(tasks)}】")
                result = self.execute_single_task(task["task"], tools, execute_tool_fn, previous_result)
                results.append(result)

                # 保存结果供下一步使用
                if result.get("success") and result.get("result"):
                    previous_result = result["result"].get("result")
                else:
                    previous_result = None

                if not result.get("success"):
                    print(f"\n⚠️  [警告] 步骤 {idx} 失败，但继续执行后续任务")

            print("\n" + "█"*60)
            print("✅ [执行完成] 任务总结")
            print("█"*60)

            for idx, (task, result) in enumerate(zip(tasks, results), 1):
                status = "✅ 成功" if result.get("success") else "❌ 失败"
                print(f"  {idx}. {task['task']} - {status}")

            return results

        except Exception as e:
            print(f"\n❌ [错误] {type(e).__name__}: {e}")
            import traceback
            traceback.print_exc()
            return []


# 便捷函数（向后兼容）
def create_llm_agent(api_key: str,
                     base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1",
                     model: str = "qwen3-32b",
                     prompt_path: str = None,
                     enable_vlm: bool = True,
                     enable_adaptive: bool = False) -> LLMAgent:
    """
    创建LLM代理实例

    Args:
        api_key: API密钥
        base_url: API基础URL
        model: 文本LLM模型名称
        prompt_path: 规划提示词文件路径
        enable_vlm: 是否启用VLM环境理解
        enable_adaptive: 是否启用自适应控制

    Returns:
        LLMAgent实例
    """
    return LLMAgent(
        api_key=api_key,
        base_url=base_url,
        model=model,
        prompt_path=prompt_path,
        enable_vlm=enable_vlm,
        enable_adaptive=enable_adaptive
    )
