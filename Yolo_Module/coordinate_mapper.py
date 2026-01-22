#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
坐标映射工具

处理屏幕坐标和仿真坐标之间的转换
"""

import sys
from typing import Tuple, Optional
from dataclasses import dataclass


@dataclass
class WindowOffset:
    """窗口偏移量"""
    x: int = 0
    y: int = 0  # 标题栏高度

    def __iter__(self):
        return iter((self.x, self.y))


class CoordinateMapper:
    """坐标映射器"""

    def __init__(self, window_offset: Tuple[int, int] = (0, 0)):
        """
        初始化坐标映射器

        Args:
            window_offset: 窗口偏移量 (x, y)
                          x: 左边框宽度
                          y: 上边框+标题栏高度
        """
        self.window_offset = window_offset

    def screen_to_sim(self, screen_x: float, screen_y: float) -> Tuple[float, float]:
        """
        屏幕坐标转仿真坐标

        Args:
            screen_x, screen_y: 屏幕坐标（相对于窗口左上角）

        Returns:
            (sim_x, sim_y) 仿真坐标

        Example:
            >>> mapper = CoordinateMapper(window_offset=(8, 30))
            >>> mapper.screen_to_sim(708, 330)
            (700, 300)
        """
        sim_x = screen_x - self.window_offset[0]
        sim_y = screen_y - self.window_offset[1]

        # 限制在仿真器范围内
        sim_x = max(0, min(800, sim_x))
        sim_y = max(0, min(600, sim_y))

        return sim_x, sim_y

    def sim_to_screen(self, sim_x: float, sim_y: float) -> Tuple[float, float]:
        """
        仿真坐标转屏幕坐标

        Args:
            sim_x, sim_y: 仿真坐标

        Returns:
            (screen_x, screen_y) 屏幕坐标

        Example:
            >>> mapper = CoordinateMapper(window_offset=(8, 30))
            >>> mapper.sim_to_screen(700, 300)
            (708, 330)
        """
        screen_x = sim_x + self.window_offset[0]
        screen_y = sim_y + self.window_offset[1]

        return screen_x, screen_y

    def set_window_offset(self, offset: Tuple[int, int]):
        """设置窗口偏移量"""
        self.window_offset = offset

    def auto_detect_offset(self, window_title: str = "增强版 2D Robot Simulator") -> bool:
        """
        自动检测窗口偏移量

        通过获取窗口位置和大小来计算偏移
        注意：此功能仅支持 Windows 系统（需要 pywin32）

        Args:
            window_title: 窗口标题

        Returns:
            是否成功检测
        """
        import platform
        if platform.system() != "Windows":
            print("[CoordinateMapper] 自动检测窗口偏移仅支持 Windows，将使用默认偏移", file=sys.stderr)
            return False

        try:
            import win32gui
        except ImportError:
            print("[CoordinateMapper] 未安装 pywin32，将使用默认偏移", file=sys.stderr)
            return False

        hwnd = win32gui.FindWindow(None, window_title)

        if hwnd == 0:
            print(f"[CoordinateMapper] 未找到窗口: {window_title}", file=sys.stderr)
            return False

        # 获取窗口位置
        left, top, right, bottom = win32gui.GetWindowRect(hwnd)

        # 获取客户区位置
        try:
            rect = win32gui.GetClientRect(hwnd)
            client_left, client_top, client_right, client_bottom = rect

            # 计算偏移
            offset_x = left - client_left
            offset_y = top - client_top

            self.window_offset = (offset_x, offset_y)

            print(f"[CoordinateMapper] 自动检测偏移: {self.window_offset}", file=sys.stderr)
            return True

        except Exception as e:
            print(f"[CoordinateMapper] 检测失败: {e}", file=sys.stderr)
            return False


# 默认映射器实例
_default_mapper: Optional[CoordinateMapper] = None


def get_default_mapper() -> CoordinateMapper:
    """获取默认映射器"""
    global _default_mapper
    if _default_mapper is None:
        _default_mapper = CoordinateMapper()
    return _default_mapper


def screen_to_sim(screen_x: float, screen_y: float,
                  mapper: Optional[CoordinateMapper] = None) -> Tuple[float, float]:
    """
    屏幕坐标转仿真坐标（便捷函数）

    Args:
        screen_x, screen_y: 屏幕坐标
        mapper: 坐标映射器（可选，使用默认实例如果为 None）

    Returns:
        (sim_x, sim_y)
    """
    if mapper is None:
        mapper = get_default_mapper()
    return mapper.screen_to_sim(screen_x, screen_y)


def sim_to_screen(sim_x: float, sim_y: float,
                  mapper: Optional[CoordinateMapper] = None) -> Tuple[float, float]:
    """
    仿真坐标转屏幕坐标（便捷函数）

    Args:
        sim_x, sim_y: 仿真坐标
        mapper: 坐标映射器（可选，使用默认实例如果为 None）

    Returns:
        (screen_x, screen_y)
    """
    if mapper is None:
        mapper = get_default_mapper()
    return mapper.sim_to_screen(sim_x, sim_y)


# 测试函数

def test_coordinate_mapping():
    """测试坐标映射"""
    print("="*60, file=sys.stderr)
    print("坐标映射测试", file=sys.stderr)
    print("="*60, file=sys.stderr)

    # 创建映射器（假设窗口偏移为 8, 30）
    mapper = CoordinateMapper(window_offset=(8, 30))

    # 测试用例
    test_cases = [
        {
            "name": "屏幕坐标转仿真坐标",
            "screen": (708, 330),
            "expected_sim": (700, 300)
        },
        {
            "name": "仿真坐标转屏幕坐标",
            "sim": (700, 300),
            "expected_screen": (708, 330)
        }
    ]

    import sys
    all_passed = True

    for case in test_cases:
        print(f"\n{case['name']}", file=sys.stderr)

        if "screen" in case and "expected_sim" in case:
            screen_x, screen_y = case["screen"]
            expected_x, expected_y = case["expected_sim"]

            sim_x, sim_y = mapper.screen_to_sim(screen_x, screen_y)

            if abs(sim_x - expected_x) < 0.1 and abs(sim_y - expected_y) < 0.1:
                print(f"  [PASS] ({screen_x}, {screen_y}) -> ({sim_x}, {sim_y})", file=sys.stderr)
            else:
                print(f"  [FAIL] 预期: ({expected_x}, {expected_y}) "
                      f"实际: ({sim_x}, {sim_y})", file=sys.stderr)
                all_passed = False

        elif "sim" in case and "expected_screen" in case:
            sim_x, sim_y = case["sim"]
            expected_x, expected_y = case["expected_screen"]

            screen_x, screen_y = mapper.sim_to_screen(sim_x, sim_y)

            if abs(screen_x - expected_x) < 0.1 and abs(screen_y - expected_y) < 0.1:
                print(f"  [PASS] ({sim_x}, {sim_y}) -> ({screen_x}, {screen_y})", file=sys.stderr)
            else:
                print(f"  [FAIL] 预期: ({expected_x}, {expected_y}) "
                      f"实际: ({screen_x}, {screen_y})", file=sys.stderr)
                all_passed = False

    print("\n" + "="*60, file=sys.stderr)
    if all_passed:
        print("[SUCCESS] 所有测试通过", file=sys.stderr)
    else:
        print("[FAILED] 部分测试失败", file=sys.stderr)

    return all_passed


if __name__ == "__main__":
    import sys
    success = test_coordinate_mapping()
    sys.exit(0 if success else 1)
