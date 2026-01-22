#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
屏幕截图工具

支持全屏截图和区域截图。
"""

import sys
import numpy as np
from datetime import datetime


class ScreenCapture:
    """屏幕截图工具"""

    def __init__(self):
        """初始化屏幕截图工具"""
        self.pyautogui_available = False
        self.mss_available = False
        self._check_dependencies()

    def _check_dependencies(self):
        """检查依赖项"""
        try:
            import pyautogui
            self.pyautogui_available = True
        except ImportError:
            pass

        try:
            import mss
            self.mss_available = True
        except ImportError:
            pass

        if not self.pyautogui_available and not self.mss_available:
            print("[Capture] 警告: 未安装截图依赖", file=sys.stderr)
            print("[Capture] 请安装: pip install pyautogui 或 pip install mss", file=sys.stderr)

    def capture_screen(self):
        """
        截取整个屏幕

        Returns:
            numpy 数组格式的图像 (RGB)
        """
        if self.pyautogui_available:
            return self._capture_with_pyautogui()
        elif self.mss_available:
            return self._capture_with_mss()
        else:
            raise RuntimeError("未安装截图依赖 (pyautogui 或 mss)")

    def capture_region(self, x, y, width, height):
        """
        截取屏幕指定区域

        Args:
            x, y: 区域左上角坐标
            width, height: 区域宽高

        Returns:
            numpy 数组格式的图像 (RGB)
        """
        if self.pyautogui_available:
            return self._capture_region_with_pyautogui(x, y, width, height)
        elif self.mss_available:
            return self._capture_region_with_mss(x, y, width, height)
        else:
            raise RuntimeError("未安装截图依赖 (pyautogui 或 mss)")

    def _capture_with_pyautogui(self):
        """使用 pyautogui 截取全屏"""
        import pyautogui
        screenshot = pyautogui.screenshot()
        image_array = np.array(screenshot)
        return image_array

    def _capture_region_with_pyautogui(self, x, y, width, height):
        """使用 pyautogui 截取区域"""
        import pyautogui
        screenshot = pyautogui.screenshot(region=(x, y, width, height))
        image_array = np.array(screenshot)
        return image_array

    def _capture_with_mss(self):
        """使用 mss 截取全屏"""
        import mss
        with mss.mss() as sct:
            monitor = sct.monitors[0]  # 主显示器
            screenshot = sct.grab(monitor)
            image_array = np.array(screenshot)
            # BGRA 转 RGB
            image_array = image_array[:, :, :3][:, :, ::-1]
            return image_array

    def _capture_region_with_mss(self, x, y, width, height):
        """使用 mss 截取区域"""
        import mss
        with mss.mss() as sct:
            monitor = {"top": y, "left": x, "width": width, "height": height}
            screenshot = sct.grab(monitor)
            image_array = np.array(screenshot)
            # BGRA 转 RGB
            image_array = image_array[:, :, :3][:, :, ::-1]
            return image_array

    @staticmethod
    def save_image(image_array, filename):
        """
        保存图像到文件

        Args:
            image_array: numpy 数组格式的图像 (RGB)
            filename: 保存路径
        """
        try:
            import cv2
            cv2.imwrite(filename, cv2.cvtColor(image_array, cv2.COLOR_RGB2BGR))
            print(f"[Capture] 图像已保存: {filename}", file=sys.stderr)
        except ImportError:
            print("[Capture] 错误: 未安装 opencv-python", file=sys.stderr)
            print("[Capture] 请运行: pip install opencv-python", file=sys.stderr)
            raise

    @staticmethod
    def save_image_with_timestamp(image_array, directory="/tmp", prefix="capture"):
        """
        保存图像到文件，文件名包含时间戳

        Args:
            image_array: numpy 数组格式的图像 (RGB)
            directory: 保存目录
            prefix: 文件名前缀

        Returns:
            保存的文件路径
        """
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{directory}/{prefix}_{timestamp}.jpg"
        ScreenCapture.save_image(image_array, filename)
        return filename


def main():
    """测试函数"""
    import cv2
    import time

    print("[测试] 屏幕截图工具", file=sys.stderr)

    # 初始化截图工具
    capturer = ScreenCapture()

    # 截取全屏
    print("[截图] 正在截取全屏...", file=sys.stderr)
    image = capturer.capture_screen()
    print(f"[截图] 图像尺寸: {image.shape}", file=sys.stderr)

    # 保存图像
    filename = ScreenCapture.save_image_with_timestamp(image, prefix="test_fullscreen")
    print(f"[截图] 已保存: {filename}", file=sys.stderr)

    # 等待 3 秒
    print("[截图] 3 秒后截取区域...", file=sys.stderr)
    time.sleep(3)

    # 截取区域
    region = (100, 100, 800, 600)
    print(f"[截图] 截取区域: {region}", file=sys.stderr)
    region_image = capturer.capture_region(*region)
    print(f"[截图] 图像尺寸: {region_image.shape}", file=sys.stderr)

    # 保存区域图像
    region_filename = ScreenCapture.save_image_with_timestamp(region_image, prefix="test_region")
    print(f"[截图] 已保存: {region_filename}", file=sys.stderr)

    print("[测试] 完成", file=sys.stderr)


if __name__ == "__main__":
    main()
