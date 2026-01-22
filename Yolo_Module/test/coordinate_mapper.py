#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
坐标映射工具

用于将屏幕坐标映射到仿真器坐标。
"""

import sys


class CoordinateMapper:
    """坐标映射器"""

    def __init__(self, screen_region=None, simulator_size=None):
        """
        初始化坐标映射器

        Args:
            screen_region: 屏幕区域 (x, y, width, height)
            simulator_size: 仿真器尺寸 (width, height)
        """
        self.screen_region = screen_region
        self.simulator_size = simulator_size

    def set_screen_region(self, screen_region):
        """设置屏幕区域"""
        self.screen_region = screen_region

    def set_simulator_size(self, simulator_size):
        """设置仿真器尺寸"""
        self.simulator_size = simulator_size

    def screen_to_simulator(self, screen_x, screen_y):
        """
        屏幕坐标转仿真器坐标

        Args:
            screen_x, screen_y: 屏幕坐标（相对于屏幕区域）

        Returns:
            仿真器坐标 (sim_x, sim_y)
        """
        if self.screen_region is None or self.simulator_size is None:
            raise ValueError("请先设置 screen_region 和 simulator_size")

        # 计算相对位置 (0-1)
        rel_x = screen_x / self.screen_region[2]
        rel_y = screen_y / self.screen_region[3]

        # 映射到仿真器坐标
        sim_x = rel_x * self.simulator_size[0]
        sim_y = rel_y * self.simulator_size[1]

        return sim_x, sim_y

    def simulator_to_screen(self, sim_x, sim_y):
        """
        仿真器坐标转屏幕坐标

        Args:
            sim_x, sim_y: 仿真器坐标

        Returns:
            屏幕坐标 (screen_x, screen_y)
        """
        if self.screen_region is None or self.simulator_size is None:
            raise ValueError("请先设置 screen_region 和 simulator_size")

        # 计算相对位置 (0-1)
        rel_x = sim_x / self.simulator_size[0]
        rel_y = sim_y / self.simulator_size[1]

        # 映射到屏幕坐标
        screen_x = rel_x * self.screen_region[2] + self.screen_region[0]
        screen_y = rel_x * self.screen_region[3] + self.screen_region[1]

        return screen_x, screen_y

    def absolute_screen_to_simulator(self, abs_screen_x, abs_screen_y):
        """
        绝对屏幕坐标转仿真器坐标

        Args:
            abs_screen_x, abs_screen_y: 绝对屏幕坐标

        Returns:
            仿真器坐标 (sim_x, sim_y)
        """
        if self.screen_region is None or self.simulator_size is None:
            raise ValueError("请先设置 screen_region 和 simulator_size")

        # 转换为相对于屏幕区域的坐标
        rel_screen_x = abs_screen_x - self.screen_region[0]
        rel_screen_y = abs_screen_y - self.screen_region[1]

        # 映射到仿真器坐标
        return self.screen_to_simulator(rel_screen_x, rel_screen_y)


def main():
    """测试函数"""
    print("[测试] 坐标映射器", file=sys.stderr)

    # 初始化映射器
    screen_region = (100, 100, 800, 600)  # x, y, width, height
    simulator_size = (800, 600)  # width, height

    mapper = CoordinateMapper(screen_region, simulator_size)

    print(f"[配置] 屏幕区域: {screen_region}", file=sys.stderr)
    print(f"[配置] 仿真器尺寸: {simulator_size}", file=sys.stderr)

    # 测试屏幕坐标转仿真器坐标
    test_cases = [
        (0, 0),           # 左上角
        (400, 300),       # 中心
        (800, 600),       # 右下角
        (200, 150),       # 四分之一
    ]

    print("\n[测试] 屏幕坐标 → 仿真器坐标:", file=sys.stderr)
    for sx, sy in test_cases:
        sim_x, sim_y = mapper.screen_to_simulator(sx, sy)
        print(f"  屏幕({sx:4d}, {sy:4d}) → 仿真器({sim_x:7.1f}, {sim_y:7.1f})", file=sys.stderr)

    # 测试仿真器坐标转屏幕坐标
    print("\n[测试] 仿真器坐标 → 屏幕坐标:", file=sys.stderr)
    sim_test_cases = [
        (0, 0),
        (400, 300),
        (800, 600),
        (200, 150),
    ]

    for sim_x, sim_y in sim_test_cases:
        screen_x, screen_y = mapper.simulator_to_screen(sim_x, sim_y)
        print(f"  仿真器({sim_x:7.1f}, {sim_y:7.1f}) → 屏幕({screen_x:7.1f}, {screen_y:7.1f})", file=sys.stderr)

    # 测试绝对屏幕坐标
    print("\n[测试] 绝对屏幕坐标 → 仿真器坐标:", file=sys.stderr)
    abs_test_cases = [
        (100, 100),       # 屏幕区域左上角
        (500, 400),       # 屏幕区域中心
        (900, 700),       # 屏幕区域右下角
    ]

    for abs_x, abs_y in abs_test_cases:
        sim_x, sim_y = mapper.absolute_screen_to_simulator(abs_x, abs_y)
        print(f"  绝对屏幕({abs_x:4d}, {abs_y:4d}) → 仿真器({sim_x:7.1f}, {sim_y:7.1f})", file=sys.stderr)

    print("\n[测试] 完成", file=sys.stderr)


if __name__ == "__main__":
    main()
