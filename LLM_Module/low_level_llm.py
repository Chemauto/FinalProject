#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
低层LLM - 任务执行控制器
负责选择工具、生成参数、执行任务、监控执行状态
"""
import json
import sys
import time
from openai import OpenAI
from typing import Callable, Dict, List, Any, Optional
from enum import Enum


class ExecutionStatus(Enum):
    """执行状态枚举"""
    SUCCESS = "success"
    FAILED = "failed"
    PARTIAL = "partial"
    REQUIRES_REPLANNING = "requires_replanning"


class LowLevelLLM:
    """
    低层LLM：任务执行控制器

    职责：
    1. 选择执行当前任务的最佳技能/工具
    2. 生成技能参数
    3. 融合实时感知信息
    4. 监控执行状态并反馈
    5. 检测环境变化
    """

    def __init__(self,
                 api_key: str,
                 base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1",
                 model: str = "qwen3-32b"):
        """
        初始化低层LLM

        Args:
            api_key: API密钥
            base_url: API基础URL
            model: 使用的模型名称
        """
        self.client = OpenAI(api_key=api_key, base_url=base_url)
        self.model = model
        self.system_prompt = self._build_system_prompt()

    def _build_system_prompt(self) -> str:
        """构建系统提示词"""
        return """你是一个机器人控制助手。根据子任务描述，调用相应的工具函数。

【关键规则 - 强制执行】
1. **追击敌人任务（最重要）**：
   - 如果任务描述包含"追击"字样，必须调用 chase_enemy(enemy_positions=...)
   - enemy_positions 参数必须使用"上一步的结果"中的值
   - 即使上一步的结果是空列表 []，也要传递给 chase_enemy！
   - **绝对禁止**在追击任务中调用 get_enemy_positions()

2. **正确示例**：
   - 任务："获取敌人位置" → 调用 get_enemy_positions()
   - 任务："追击最近的敌人" + 上一步结果='[{"id":"1","x":100,"y":200}]' → 调用 chase_enemy(enemy_positions='[{"id":"1","x":100,"y":200}]')
   - 任务："追击最近的敌人" + 上一步结果='[]' → 调用 chase_enemy(enemy_positions='[]')

3. **错误示例（不要这样做）**：
   - 任务："追击敌人" → 调用 get_enemy_positions()  ❌ 错误！
   - 任务："追击敌人" → 调用 chase_enemy()  ❌ 缺少参数！

4. **图片路径参数**：
   - 如果任务描述中包含文件路径（.png, .jpg），必须将其作为参数传入
   - 调用 detect_color_and_act 时，必须设置 image_path 参数

【执行决策】
- 如果有"上一步的结果"且任务是"追击" → chase_enemy(enemy_positions=上一步的结果)
- 如果任务是"获取敌人位置" → get_enemy_positions()
- 其他任务 → 根据描述调用相应工具

记住：chase_enemy 需要一个 JSON 字符串参数，不要调用它时省略参数！"""

    def execute_task(self,
                     task_description: str,
                     tools: List[Dict],
                     execute_tool_fn: Callable,
                     previous_result: Any = None,
                     perception_data: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        执行单个任务

        Args:
            task_description: 任务描述
            tools: 可用工具列表（OpenAI function calling格式）
            execute_tool_fn: 工具执行函数
            previous_result: 上一步的执行结果
            perception_data: 当前感知数据（用于环境变化检测）

        Returns:
            执行结果字典，包含：
            - status: 执行状态 (success/failed/partial/requires_replanning)
            - action: 执行的动作（工具名称）
            - task: 任务描述
            - result: 工具执行结果
            - error: 错误信息（如果有）
        """
        print(f"\n{'─'*50}")
        print(f"⚙️  [低层LLM] 执行任务: {task_description}")
        print(f"{'─'*50}")

        # 检查环境变化（如果有感知数据）
        if perception_data and self._detect_environment_change(perception_data):
            print("⚠️  [环境变化检测] 检测到环境变化，需要重新规划")
            return {
                "status": ExecutionStatus.REQUIRES_REPLANNING.value,
                "reason": "environment_changed",
                "message": "环境已变化，需要重新规划"
            }

        # 构建用户消息
        user_message = f"执行任务：{task_description}"

        if previous_result is not None:
            user_message += f"\n\n上一步的结果：{previous_result}"
            print(f"[低层LLM] 上一步结果: {previous_result}", file=sys.stderr)
        else:
            print(f"[低层LLM] 没有上一步结果", file=sys.stderr)

        # 如果有感知数据，添加到消息中
        if perception_data:
            user_message += f"\n\n当前感知信息：{json.dumps(perception_data, indent=2, ensure_ascii=False)}"

        try:
            # 调用LLM选择工具
            completion = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": user_message}
                ],
                tools=tools,
                tool_choice="auto",
                extra_body={"enable_thinking": False}
            )

            response_message = completion.choices[0].message
            tool_calls = response_message.tool_calls

            # 检查是否有工具调用
            if not tool_calls:
                print("[跳过] 无效指令或无法识别的操作")
                return {
                    "status": ExecutionStatus.FAILED.value,
                    "action": "none",
                    "task": task_description,
                    "error": "No tool called"
                }

            # 执行工具调用
            tool_call = tool_calls[0]
            function_name = tool_call.function.name
            function_args = json.loads(tool_call.function.arguments)

            print(f"🔧 [工具调用] {function_name}({function_args})")

            # 执行工具
            result = execute_tool_fn(function_name, function_args)

            # 处理延迟（如果工具返回了delay信息）
            if result and result.get("delay"):
                self._wait_for_completion(result["delay"])

            # 检查执行结果
            if result and isinstance(result, dict):
                if result.get("success") is False:
                    return {
                        "status": ExecutionStatus.FAILED.value,
                        "action": function_name,
                        "task": task_description,
                        "result": result,
                        "error": result.get("error", "Unknown error")
                    }

                return {
                    "status": ExecutionStatus.SUCCESS.value,
                    "action": function_name,
                    "task": task_description,
                    "result": result
                }

            return {
                "status": ExecutionStatus.SUCCESS.value,
                "action": function_name,
                "task": task_description,
                "result": result
            }

        except Exception as e:
            print(f"\n❌ [错误] 执行失败: {e}")
            import traceback
            traceback.print_exc()

            return {
                "status": ExecutionStatus.FAILED.value,
                "error": str(e),
                "task": task_description
            }

    def _detect_environment_change(self, perception_data: Dict[str, Any]) -> bool:
        """
        检测环境是否发生变化

        Args:
            perception_data: 感知数据

        Returns:
            是否检测到环境变化
        """
        # 检查感知数据中的变化标记
        if perception_data.get("environment_changed"):
            return True

        # 检查目标物体是否消失
        if perception_data.get("target_missing"):
            return True

        # 检查是否有新的障碍物
        if perception_data.get("new_obstacles"):
            return True

        return False

    def _wait_for_completion(self, delay: float):
        """
        等待执行完成

        Args:
            delay: 延迟时间（秒）
        """
        print(f"⏳ [等待] 执行时间: {delay:.1f}秒", end="", flush=True)
        steps = max(1, int(delay))
        for i in range(steps):
            time.sleep(delay / steps)
            print(".", end="", flush=True)
        print(" ✅ 完成!")

    def select_skill(self,
                     task_description: str,
                     available_skills: List[Dict[str, Any]],
                     perception_data: Optional[Dict[str, Any]] = None) -> Optional[str]:
        """
        根据任务和感知数据选择最佳技能（用于动态技能选择）

        Args:
            task_description: 任务描述
            available_skills: 可用技能列表
            perception_data: 感知数据

        Returns:
            选择的技能名称
        """
        # 这个方法可以用于将来的动态技能选择
        # 当前实现使用function calling，所以这个方法暂时不使用
        pass

    def generate_parameters(self,
                           task: Dict[str, Any],
                           skill_name: str,
                           perception_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        生成技能参数（用于动态参数生成）

        Args:
            task: 任务信息
            skill_name: 技能名称
            perception_data: 感知数据

        Returns:
            生成的参数字典
        """
        # 这个方法可以用于将来的动态参数生成
        # 当前实现使用function calling，所以这个方法暂时不使用
        pass
