#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
追击核心算法单元测试

测试追击控制器的数学计算是否正确
"""

import sys
import math
from pathlib import Path

# 添加项目路径
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from Test_Module.chase_core import ChaseController


def test_calculate_target_angle():
    """测试目标角度计算"""
    print("\n" + "="*60)
    print("测试 1: 计算目标角度")
    print("="*60)

    controller = ChaseController()

    # 测试用例
    test_cases = [
        {
            "name": "目标在正东方",
            "robot": (100, 300),
            "target": (700, 300),
            "expected": 0.0  # 东
        },
        {
            "name": "目标在正北方",
            "robot": (400, 500),
            "target": (400, 100),
            "expected": 90.0  # 北
        },
        {
            "name": "目标在正西方",
            "robot": (700, 300),
            "target": (100, 300),
            "expected": 180.0  # 西
        },
        {
            "name": "目标在正南方",
            "robot": (400, 100),
            "target": (400, 500),
            "expected": 270.0  # 南
        },
        {
            "name": "目标在东北方向（45度）",
            "robot": (100, 400),
            "target": (500, 0),  # dx=400, dy=-400
            "expected": 45.0  # 东北
        },
        {
            "name": "目标在东南方向（315度）",
            "robot": (100, 0),
            "target": (500, 400),  # dx=400, dy=400
            "expected": 315.0  # 东南
        }
    ]

    all_passed = True

    for case in test_cases:
        robot_x, robot_y = case["robot"]
        target_x, target_y = case["target"]
        expected = case["expected"]

        result = controller.calculate_target_angle(robot_x, robot_y, target_x, target_y)
        diff = abs(result - expected)

        passed = diff < 1.0  # 允许1度误差
        status = "[PASS]" if passed else "[FAIL]"

        print(f"\n{status} - {case['name']}")
        print(f"  机器人: {case['robot']}")
        print(f"  目标: {case['target']}")
        print(f"  预期角度: {expected}°")
        print(f"  计算角度: {result:.1f}°")
        print(f"  误差: {diff:.1f}°")

        if not passed:
            all_passed = False

    return all_passed


def test_calculate_angle_difference():
    """测试角度差计算"""
    print("\n" + "="*60)
    print("测试 2: 计算角度差")
    print("="*60)

    controller = ChaseController()

    test_cases = [
        {
            "name": "目标在右侧（小角度）",
            "current": 0,
            "target": 45,
            "expected": 45,
            "direction": "左转"
        },
        {
            "name": "目标在左侧（小角度）",
            "current": 90,
            "target": 45,
            "expected": -45,
            "direction": "右转"
        },
        {
            "name": "跨越0度（向左）",
            "current": 350,
            "target": 10,
            "expected": 20,
            "direction": "左转"
        },
        {
            "name": "跨越0度（向右）",
            "current": 10,
            "target": 350,
            "expected": -20,
            "direction": "右转"
        },
        {
            "name": "接近180度",
            "current": 0,
            "target": 179,
            "expected": 179,
            "direction": "左转"
        },
        {
            "name": "超过180度（选择更短路径）",
            "current": 0,
            "target": 181,
            "expected": -179,
            "direction": "右转"
        }
    ]

    all_passed = True

    for case in test_cases:
        current = case["current"]
        target = case["target"]
        expected = case["expected"]

        result = controller.calculate_angle_difference(current, target)
        diff = abs(result - expected)

        passed = diff < 0.1
        status = "[PASS]" if passed else "[FAIL]"

        print(f"\n{status} - {case['name']}")
        print(f"  当前角度: {current}°")
        print(f"  目标角度: {target}°")
        print(f"  预期差值: {expected}° ({case['direction']})")
        print(f"  计算差值: {result:.1f}°")

        direction = "左转" if result > 0 else "右转" if result < 0 else "保持"
        print(f"  方向: {direction}")

        if not passed:
            all_passed = False

    return all_passed


def test_calculate_distance():
    """测试距离计算"""
    print("\n" + "="*60)
    print("测试 3: 计算距离")
    print("="*60)

    controller = ChaseController()

    test_cases = [
        {
            "name": "水平距离",
            "p1": (0, 0),
            "p2": (300, 0),
            "expected": 300.0
        },
        {
            "name": "垂直距离",
            "p1": (0, 0),
            "p2": (0, 400),
            "expected": 400.0
        },
        {
            "name": "对角线距离（3-4-5三角形）",
            "p1": (0, 0),
            "p2": (300, 400),
            "expected": 500.0
        },
        {
            "name": "相同点",
            "p1": (100, 100),
            "p2": (100, 100),
            "expected": 0.0
        }
    ]

    all_passed = True

    for case in test_cases:
        x1, y1 = case["p1"]
        x2, y2 = case["p2"]
        expected = case["expected"]

        result = controller.calculate_distance(x1, y1, x2, y2)
        diff = abs(result - expected)

        passed = diff < 0.1
        status = "[PASS]" if passed else "[FAIL]"

        print(f"\n{status} - {case['name']}")
        print(f"  点1: {case['p1']}")
        print(f"  点2: {case['p2']}")
        print(f"  预期距离: {expected} 像素")
        print(f"  计算距离: {result:.1f} 像素")
        print(f"  误差: {diff:.1f} 像素")

        if not passed:
            all_passed = False

    return all_passed


def test_complete_chase_scenario():
    """测试完整追击场景"""
    print("\n" + "="*60)
    print("测试 4: 完整追击场景模拟")
    print("="*60)

    controller = ChaseController()

    # 场景：机器人在(100, 300)，朝向东方(0°)，目标在(700, 300)
    robot_pos = {"x": 100, "y": 300, "angle": 0}
    target_pos = {"x": 700, "y": 300}

    print(f"\n初始状态:")
    print(f"  机器人位置: ({robot_pos['x']}, {robot_pos['y']})")
    print(f"  机器人角度: {robot_pos['angle']}° (东方)")
    print(f"  目标位置: ({target_pos['x']}, {target_pos['y']})")

    # 计算目标角度
    target_angle = controller.calculate_target_angle(
        robot_pos['x'], robot_pos['y'],
        target_pos['x'], target_pos['y']
    )

    print(f"\n计算结果:")
    print(f"  目标方向: {target_angle:.1f}°")

    # 计算角度差
    angle_diff = controller.calculate_angle_difference(
        robot_pos['angle'], target_angle
    )

    print(f"  角度差: {angle_diff:.1f}°")

    direction = "左转" if angle_diff > 0 else "右转" if angle_diff < 0 else "保持"
    print(f"  动作: {direction}")

    # 计算距离
    distance_pixels = controller.calculate_distance(
        robot_pos['x'], robot_pos['y'],
        target_pos['x'], target_pos['y']
    )
    distance_meters = controller.distance_to_meters(distance_pixels)

    print(f"\n距离信息:")
    print(f"  像素距离: {distance_pixels:.1f} px")
    print(f"  米制距离: {distance_meters:.2f} m")

    # 判断是否需要旋转
    needs_rotation = abs(angle_diff) > 5
    print(f"\n决策:")
    print(f"  需要旋转: {'是' if needs_rotation else '否'}")

    if not needs_rotation:
        steps_needed = int(distance_meters / 0.5)  # 每步0.5米
        print(f"  预计步数: 约 {steps_needed} 步")

    # 验证结果
    assert abs(target_angle - 0) < 1, "目标角度应该是0°（东方）"
    assert abs(angle_diff) < 1, "角度差应该接近0"
    assert abs(distance_meters - 6.0) < 0.1, "距离应该是6米"

    print(f"\n[PASS] - 场景验证通过")

    return True


def main():
    """运行所有测试"""
    print("\n" + "="*60)
    print("追击核心算法单元测试")
    print("="*60)

    tests = [
        ("计算目标角度", test_calculate_target_angle),
        ("计算角度差", test_calculate_angle_difference),
        ("计算距离", test_calculate_distance),
        ("完整追击场景", test_complete_chase_scenario)
    ]

    results = []

    for name, test_func in tests:
        try:
            passed = test_func()
            results.append((name, passed))
        except Exception as e:
            print(f"\n[ERROR] - {name}: {e}")
            import traceback
            traceback.print_exc()
            results.append((name, False))

    # 汇总结果
    print("\n" + "="*60)
    print("测试结果汇总")
    print("="*60)

    passed_count = sum(1 for _, passed in results if passed)
    total_count = len(results)

    for name, passed in results:
        status = "[PASS]" if passed else "[FAIL]"
        print(f"{status} - {name}")

    print("\n" + "="*60)
    print(f"总计: {passed_count}/{total_count} 个测试通过")

    if passed_count == total_count:
        print("[SUCCESS] 所有测试通过！")
        return 0
    else:
        print("[FAILED] 部分测试失败")
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
