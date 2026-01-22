#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
屏幕截图工具

用于截取仿真器窗口画面供 YOLO 检测
"""

import sys
import time
from pathlib import Path
from typing import Optional, Tuple


def capture_simulator(
    window_title: str = "增强版 2D Robot Simulator",
    save_path: Optional[str] = None
) -> Optional['PIL.Image.Image']:
    """
    截取仿真器窗口

    Args:
        window_title: 窗口标题
        save_path: 保存路径（可选）

    Returns:
        PIL Image 对象，失败返回 None
    """
    try:
        import PIL.ImageGrab
        import win32gui
        import win32con
    except ImportError as e:
        print(f"[screen_capture] 缺少依赖: {e}", file=sys.stderr)
        print("  请运行: pip install pillow pywin32", file=sys.stderr)
        return None

    # 查找窗口
    hwnd = win32gui.FindWindow(None, window_title)

    if hwnd == 0:
        print(f"[screen_capture] 未找到窗口: {window_title}", file=sys.stderr)

        # 尝试备选标题
        alternative_titles = [
            "2D Robot Simulator",
            "Simulator",
            "Sim",
        ]

        for alt_title in alternative_titles:
            hwnd = win32gui.FindWindow(None, alt_title)
            if hwnd != 0:
                print(f"[screen_capture] 使用备选窗口: {alt_title}", file=sys.stderr)
                break

        if hwnd == 0:
            return None

    try:
        # 获取窗口位置
        left, top, right, bottom = win32gui.GetWindowRect(hwnd)
        width = right - left
        height = bottom - top

        print(f"[screen_capture] 窗口位置: ({left}, {top}) 大小: {width}x{height}", file=sys.stderr)

        # 截图
        screenshot = PIL.ImageGrab.grab(bbox=(left, top, right, bottom))

        # 保存
        if save_path:
            screenshot.save(save_path)
            print(f"[screen_capture] 已保存: {save_path}", file=sys.stderr)

        return screenshot

    except Exception as e:
        print(f"[screen_capture] 截图失败: {e}", file=sys.stderr)
        return None


def capture_full_screen(save_path: Optional[str] = None) -> Optional['PIL.Image.Image']:
    """
    截取整个屏幕

    Args:
        save_path: 保存路径（可选）

    Returns:
        PIL Image 对象，失败返回 None
    """
    try:
        import PIL.ImageGrab
    except ImportError:
        print("[screen_capture] 缺少依赖: pillow", file=sys.stderr)
        print("  请运行: pip install pillow", file=sys.stderr)
        return None

    try:
        screenshot = PIL.ImageGrab.grab()

        if save_path:
            screenshot.save(save_path)
            print(f"[screen_capture] 全屏截图已保存: {save_path}", file=sys.stderr)

        return screenshot

    except Exception as e:
        print(f"[screen_capture] 全屏截图失败: {e}", file=sys.stderr)
        return None


def save_screenshot(image: 'PIL.Image.Image', filename: str = None) -> str:
    """
    保存截图

    Args:
        image: PIL Image 对象
        filename: 文件名（可选，默认使用时间戳）

    Returns:
        保存的文件路径
    """
    if filename is None:
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"screenshot_{timestamp}.png"

    # 保存到 screenshots 目录
    save_dir = Path(__file__).parent.parent / "screenshots"
    save_dir.mkdir(exist_ok=True)

    save_path = save_dir / filename
    image.save(str(save_path))

    print(f"[screen_capture] 已保存: {save_path}", file=sys.stderr)

    return str(save_path)


def get_window_position(window_title: str) -> Optional[Tuple[int, int, int, int]]:
    """
    获取窗口位置和大小

    Args:
        window_title: 窗口标题

    Returns:
        (left, top, right, bottom) 或 None
    """
    try:
        import win32gui
    except ImportError:
        return None

    hwnd = win32gui.FindWindow(None, window_title)

    if hwnd == 0:
        return None

    return win32gui.GetWindowRect(hwnd)


def list_windows():
    """列出所有窗口（用于调试）"""
    try:
        import win32gui
    except ImportError:
        print("[screen_capture] 需要 pywin32", file=sys.stderr)
        return

    def window_callback(hwnd, windows):
        if win32gui.IsWindowVisible(hwnd):
            title = win32gui.GetWindowText(hwnd)
            if title:
                windows.append((hwnd, title))
        return True

    windows = []
    win32gui.EnumWindows(window_callback, windows)

    print("\n[screen_capture] 可见窗口列表:", file=sys.stderr)
    print("-" * 60, file=sys.stderr)
    for hwnd, title in windows:
        print(f"  [{hwnd}] {title}", file=sys.stderr)
    print("-" * 60, file=sys.stderr)


if __name__ == "__main__":
    """测试截图功能"""
    print("="*60, file=sys.stderr)
    print("屏幕截图工具测试", file=sys.stderr)
    print("="*60, file=sys.stderr)

    # 列出所有窗口
    print("\n1. 列出所有窗口", file=sys.stderr)
    list_windows()

    # 截取仿真器
    print("\n2. 截取仿真器窗口", file=sys.stderr)
    screenshot = capture_simulator(save_path="test_screenshot.png")

    if screenshot:
        print(f"  ✓ 成功: {screenshot.size}", file=sys.stderr)
    else:
        print("  ✗ 失败", file=sys.stderr)
        print("\n  提示: 确保仿真器正在运行", file=sys.stderr)
