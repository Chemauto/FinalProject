#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO 检测模块 - Demo 演示程序

演示如何使用目标检测模块：
1. 键盘输入坐标生成敌人
2. 鼠标点击生成敌人（配合仿真器）
3. 坐标映射测试
4. 屏幕截图（可选）
"""

import sys
import asyncio
from pathlib import Path

# 添加项目根目录到路径
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from Yolo_Module.target_detector import (
    DetectionMethod,
    create_detector
)


def print_menu():
    """打印主菜单"""
    print("\n" + "="*60)
    print("YOLO 检测模块 - Demo")
    print("="*60)
    print("\n请选择功能:")
    print("1. 键盘输入坐标生成敌人")
    print("2. 坐标映射测试")
    print("3. 查看使用说明")
    print("0. 退出")
    print("="*60)


async def demo_keyboard_input():
    """演示键盘输入检测"""
    print("\n" + "="*60)
    print("功能1: 键盘输入坐标生成敌人")
    print("="*60)
    print("\n说明: 输入坐标来设置敌人位置")
    print("格式: x,y 或 x y (例如: 700,300)")
    print("输入 'q' 取消")

    detector = create_detector(DetectionMethod.KEYBOARD)
    targets = await detector.detect()

    if targets:
        print("\n✓ 成功生成敌人:")
        for target in targets:
            print(f"  ID: {target.id}")
            print(f"  位置: ({target.x}, {target.y})")
            print(f"  置信度: {target.confidence}")
    else:
        print("\n✗ 未生成敌人")


def demo_coordinate_mapping():
    """演示坐标映射"""
    print("\n" + "="*60)
    print("功能2: 坐标映射测试")
    print("="*60)

    from Yolo_Module.coordinate_mapper import CoordinateMapper

    # 创建映射器
    mapper = CoordinateMapper(window_offset=(8, 30))

    print("\n窗口偏移量: (8, 30)")
    print("  - 左边框宽度: 8 像素")
    print("  - 标题栏高度: 30 像素\n")

    # 测试用例
    test_cases = [
        ("屏幕坐标 -> 仿真坐标", 708, 330),
        ("仿真坐标 -> 屏幕坐标", 700, 300, False),
    ]

    for case_name, x, y, *rest in test_cases:
        is_screen_to_sim = rest[0] if rest else True

        print(f"{case_name}:")

        if is_screen_to_sim:
            sim_x, sim_y = mapper.screen_to_sim(x, y)
            print(f"  屏幕 ({x}, {y}) -> 仿真 ({sim_x}, {sim_y})")
        else:
            screen_x, screen_y = mapper.sim_to_screen(x, y)
            print(f"  仿真 ({x}, {y}) -> 屏幕 ({screen_x}, {screen_y})")

    print("\n✓ 坐标映射测试完成")


def demo_screen_capture():
    """演示屏幕截图（可选）"""
    print("\n" + "="*60)
    print("功能3: 屏幕截图（可选）")
    print("="*60)

    try:
        from Yolo_Module.screen_capture import capture_simulator, list_windows

        # 列出窗口
        print("\n正在查找窗口...")
        list_windows()

        # 截图
        print("\n尝试截取仿真器窗口...")
        screenshot = capture_simulator(save_path="screenshot.png")

        if screenshot:
            print(f"✓ 截图成功: {screenshot.size}")
            print("  已保存为: screenshot.png")
        else:
            print("✗ 截图失败")
            print("  提示: 请确保仿真器正在运行")

    except ImportError as e:
        print(f"✗ 缺少依赖: {e}")
        print("  如需使用截图功能，请运行: pip install pillow pywin32")


def show_usage():
    """显示使用说明"""
    print("\n" + "="*60)
    print("使用说明")
    print("="*60)

    print("\n【方式1: 鼠标点击生成敌人】")
    print("  步骤:")
    print("    1. 启动仿真器: python Test_Module/enhanced_simulator.py")
    print("    2. 在仿真器中按 'M' 进入鼠标标记模式")
    print("    3. 点击地图任意位置生成敌人")
    print("    4. 再次按 'M' 退出标记模式")

    print("\n【方式2: 键盘输入坐标】")
    print("  - 在本程序中选择功能1")
    print("  - 按提示输入坐标 (例如: 700,300)")

    print("\n【坐标映射】")
    print("  - 屏幕坐标: 相对于窗口左上角（包含边框）")
    print("  - 仿真坐标: Pygame 内部坐标（800x600）")
    print("  - 映射函数可自动转换两种坐标")

    print("\n【仿真器操作】")
    print("  [R] - 随机生成敌人")
    print("  [M] - 进入/退出鼠标标记模式")
    print("  [C] - 清除所有敌人")
    print("  [L] - 切换追击线显示")
    print("  [ESC] - 退出")


async def main():
    """主函数"""
    while True:
        print_menu()

        try:
            choice = input("\n请选择 (0-3): ").strip()

            if choice == "0":
                print("\n再见!")
                break

            elif choice == "1":
                await demo_keyboard_input()

            elif choice == "2":
                demo_coordinate_mapping()

            elif choice == "3":
                show_usage()

            else:
                print("\n✗ 无效选择，请重新输入")

        except KeyboardInterrupt:
            print("\n\n再见!")
            break

        except Exception as e:
            print(f"\n✗ 错误: {e}")
            import traceback
            traceback.print_exc()

        # 暂停
        input("\n按 Enter 继续...")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\n再见!")
        sys.exit(0)
