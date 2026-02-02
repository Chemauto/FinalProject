#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试监控信息显示
"""

import asyncio
import sys
from pathlib import Path

# 添加项目路径
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))


async def main():
    """测试监控显示"""
    from LLM_Module.adaptive_controller import AdaptiveController
    from LLM_Module.high_level_llm import HighLevelLLM
    from LLM_Module.low_level_llm import LowLevelLLM
    from LLM_Module.execution_monitor import EnvironmentMonitor

    # 初始化组件
    env_monitor = EnvironmentMonitor(timeout_threshold=30.0)

    # 模拟环境状态
    env_state = {
        "position": {"x": 1.0, "y": 2.0, "z": 0.0},
        "sensor_status": {
            "camera": "ok",
            "lidar": "ok",
            "imu": "ok"
        },
        "environment_version": 1
    }

    print("="*60)
    print("测试1: 正常环境状态")
    print("="*60)

    from LLM_Module.task_queue import Task, TaskStatus
    task = Task(step=1, task="测试任务", type="测试")

    # 检测环境异常
    anomaly = env_monitor.detect_anomaly(
        current_state=env_state,
        task={"task": task.task, "type": task.type}
    )

    if anomaly:
        print(f"⚠️  检测到异常: {anomaly.description}")
    else:
        print("✅ 环境状态正常")
        print(f"📍 位置: x={env_state['position']['x']:.2f}, "
              f"y={env_state['position']['y']:.2f}, "
              f"z={env_state['position']['z']:.2f}")
        print(f"🔌 传感器: {', '.join([f'{k}={v}' for k, v in env_state['sensor_status'].items()])}")

    print("\n" + "="*60)
    print("测试2: 传感器失效")
    print("="*60)

    # 模拟传感器失效
    env_state["sensor_status"]["lidar"] = "failed"

    anomaly = env_monitor.detect_anomaly(
        current_state=env_state,
        task={"task": task.task, "type": task.type}
    )

    if anomaly:
        print(f"⚠️  检测到异常: {anomaly.description}")
        print(f"📊 严重程度: {anomaly.severity}")
        print(f"📋 异常数据: {anomaly.data}")
    else:
        print("✅ 环境状态正常")

    print("\n" + "="*60)
    print("测试3: 环境变化")
    print("="*60)

    # 恢复传感器
    env_state["sensor_status"]["lidar"] = "ok"
    # 改变环境版本
    env_state["environment_version"] = 2

    anomaly = env_monitor.detect_anomaly(
        current_state=env_state,
        task={"task": task.task, "type": task.type}
    )

    if anomaly:
        print(f"⚠️  检测到异常: {anomaly.description}")
        print(f"📊 严重程度: {anomaly.severity}")
        print(f"📋 异常数据: {anomaly.data}")
    else:
        print("✅ 环境状态正常")


if __name__ == "__main__":
    asyncio.run(main())
