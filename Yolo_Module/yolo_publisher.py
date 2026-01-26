#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO 敌人位置 ROS 发布器 - 集成实时窗口检测（最终版本）
兼容当前ROS2环境，提供完整的实时检测功能
"""
import os
os.environ['YOLO_OFFLINE'] = 'True'

import json
import time
import cv2
import mss
import numpy as np
import sys
import subprocess
import signal

# 导入simulator_position模块
try:
    from simulator_position import get_current_position
    print("[YOLO] 成功导入simulator_position模块")
except ImportError as e:
    print(f"[YOLO] 导入simulator_position模块失败: {e}")
    get_current_position = None

# 尝试导入ROS2，如果失败则使用模拟模式
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
    print("[YOLO] ROS2环境可用")
except ImportError as e:
    print(f"[YOLO] ROS2环境不可用: {e}")
    print("[YOLO] 将使用模拟模式（仅显示检测结果）")
    ROS2_AVAILABLE = False

# 尝试导入YOLO
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError as e:
    print(f"[YOLO] YOLO不可用: {e}")
    YOLO_AVAILABLE = False

# 配置参数
DETECTION_FREQUENCY = 5.0  # Hz (从1Hz提升到5Hz)
CONFIDENCE_THRESHOLD = 0.9  # 置信度阈值从0.85提升到0.9
WINDOW_POSITION_UPDATE_INTERVAL = 30.0  # 窗口位置更新间隔（秒）- 降低频率避免错误位置
MAX_RETRY_ATTEMPTS = 3  # 最大重试次数
SHOW_DETECTION_WINDOW = False  # 默认不显示检测窗口，防止Qt相关的崩溃

# 获取窗口位置（集成simulator_position模块）
def get_window_position():
    """使用simulator_position模块获取窗口位置，带回退机制"""
    global get_current_position

    # 方法1: 使用simulator_position模块
    if get_current_position is not None:
        for attempt in range(MAX_RETRY_ATTEMPTS):
            try:
                position = get_current_position()
                if position and 'position' in position and 'size' in position:
                    x, y = position['position']
                    w, h = position['size']

                    # 验证窗口尺寸是否接近预期（包含窗口装饰）
                    # xwininfo 可能返回包含装饰的尺寸，但我们强制使用 800x600
                    if w >= 750 and w <= 900 and h >= 550 and h <= 700:
                        # 计算窗口装饰偏移量
                        # 窗口外框尺寸 (w, h) 包含装饰，游戏区域是 800x600
                        # 假设装饰均匀分布，计算偏移
                        offset_x = (w - 800) // 2  # 水平装饰偏移（左右边框）
                        offset_y = (h - 600) // 2  # 垂直装饰偏移（标题栏+下边框）

                        # 调整截图区域：窗口位置 + 装饰偏移
                        capture_x = x + offset_x
                        capture_y = y + offset_y

                        geometry = {"left": capture_x, "top": capture_y, "width": 800, "height": 600}
                        print(f"[YOLO] 窗口外框: {w}x{h} @ ({x},{y})，装饰偏移: ({offset_x},{offset_y})，截图区域: ({capture_x},{capture_y})")
                        return geometry
                    else:
                        print(f"[YOLO] 窗口尺寸异常: 实际{w}x{h}，期望800x600左右")
                        if attempt < MAX_RETRY_ATTEMPTS - 1:
                            time.sleep(0.5)
                            continue
                        # 尺寸不匹配但仍然尝试
                        geometry = {"left": x, "top": y, "width": 800, "height": 600}
                        return geometry

            except Exception as e:
                print(f"[YOLO] simulator_position检测尝试 {attempt + 1} 失败: {e}")
                if attempt < MAX_RETRY_ATTEMPTS - 1:
                    time.sleep(0.5)

    # 方法2: 回退到默认位置
    print("[YOLO] 使用默认窗口位置")
    return {"top": 0, "left": 0, "width": 800, "height": 600}

class YOLOEnemyPublisher:
    """YOLO敌人检测发布器"""
    
    def __init__(self):
        """初始化发布器"""
        self.node = None
        self.publisher = None
        self.model = None
        self.monitor = None
        self.sct = None
        
        # 初始化ROS2（如果可用）
        if ROS2_AVAILABLE:
            try:
                if not rclpy.ok():
                    rclpy.init()
                self.node = Node('yolo_enemy_publisher')
                self.publisher = self.node.create_publisher(String, '/robot/yolo_enemies', 10)
                print("[YOLO] ROS2发布器已创建: /robot/yolo_enemies")
            except Exception as e:
                print(f"[YOLO] ROS2初始化失败: {e}")
                self.node = None
                self.publisher = None
        
        # 加载YOLO模型
        if YOLO_AVAILABLE:
            try:
                self.model = YOLO("/home/xcj/work/FinalProject/Yolo_Module/best.pt")
                print("[YOLO] YOLO模型加载成功")
            except Exception as e:
                print(f"[YOLO] YOLO模型加载失败: {e}")
                self.model = None
        
        # 初始化屏幕截图
        try:
            self.sct = mss.mss()
            print("[YOLO] 屏幕截图初始化成功")
        except Exception as e:
            print(f"[YOLO] 屏幕截图初始化失败: {e}")
            self.sct = None
        
        # 初始窗口位置检测
        self.monitor = get_window_position()
        
        # 时间跟踪变量
        self.last_detection_time = 0
        self.last_position_update_time = 0
        self.detection_count = 0
        self.error_count = 0
        
        # 信号处理
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        self.running = True
    
    def _signal_handler(self, signum, frame):
        """信号处理器"""
        print(f"\n[YOLO] 收到信号 {signum}，正在停止...")
        self.running = False
    
    def publish_detection(self, detections):
        """发布检测结果"""
        if not self.publisher:
            # 模拟模式：仅打印结果
            print(f"[YOLO] 模拟发布: {len(detections)} 个敌人")
            return
        
        try:
            # 添加检测元数据
            detection_data = {
                "detections": detections,
                "timestamp": time.time(),
                "window_position": self.monitor,
                "detection_count": self.detection_count,
                "confidence_threshold": CONFIDENCE_THRESHOLD
            }
            
            # 发布检测结果
            msg = String()
            msg.data = json.dumps(detection_data, ensure_ascii=False)
            self.publisher.publish(msg)
            
        except Exception as e:
            print(f"[YOLO] 发布失败: {e}")
    
    def validate_monitor(self, monitor):
        """验证窗口位置是否合理，防止截图超出屏幕"""
        # 获取屏幕尺寸
        try:
            import tkinter as tk
            root = tk.Tk()
            screen_width = root.winfo_screenwidth()
            screen_height = root.winfo_screenheight()
            root.destroy()
        except:
            # 默认屏幕尺寸
            screen_width = 1920
            screen_height = 1080

        # 检查窗口是否在屏幕范围内
        left = monitor["left"]
        top = monitor["top"]
        width = monitor["width"]
        height = monitor["height"]

        # 检查是否超出屏幕边界（允许部分超出）
        if left > screen_width or top > screen_height:
            return False, f"窗口位置超出屏幕: ({left}, {top})"

        # 检查尺寸是否合理
        if width <= 0 or height <= 0 or width > 2000 or height > 2000:
            return False, f"窗口尺寸不合理: {width}x{height}"

        return True, "窗口位置有效"

    def detect_once(self):
        """进行一次检测"""
        if not self.model or not self.sct:
            return []

        # 验证窗口位置
        valid, msg = self.validate_monitor(self.monitor)
        if not valid:
            print(f"[YOLO] {msg}，跳过本次检测")
            return []

        try:
            # 进行YOLO检测
            img = np.array(self.sct.grab(self.monitor))[:, :, :3]

            # 验证图像是否有效
            if img is None or img.size == 0:
                print(f"[YOLO] 截图失败，图像为空")
                return []

            results = self.model.predict(source=img, conf=CONFIDENCE_THRESHOLD, verbose=False, device="cpu")[0]

            # 转换检测结果
            detections = []
            if results.boxes is not None:
                for i, box in enumerate(results.boxes):
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    # 注意：YOLO检测到的坐标已经是相对于截取图像的（0-800, 0-600）
                    # 不需要再加上窗口偏移量 monitor["left"] 和 monitor["top"]
                    center_x = float((x1 + x2) / 2)
                    center_y = float((y1 + y2) / 2)
                    confidence = float(box.conf[0].cpu().numpy())

                    # 验证坐标是否在合理范围内
                    if 0 <= center_x <= 800 and 0 <= center_y <= 600:
                        detections.append({
                            "id": f"yolo_{i}",
                            "x": round(center_x, 2),
                            "y": round(center_y, 2),
                            "confidence": round(confidence, 3)
                        })
                    else:
                        print(f"[YOLO] 警告：检测到超出范围的坐标 ({center_x:.1f}, {center_y:.1f})，已忽略")

            return detections

        except Exception as e:
            raise e
    
    def run(self):
        """运行检测循环"""
        if not self.model or not self.sct:
            print("[YOLO] 关键组件不可用，无法运行")
            return
        
        print(f"[YOLO] 开始运行 - 检测频率: {DETECTION_FREQUENCY}Hz, 置信度阈值: {CONFIDENCE_THRESHOLD}")
        print("[YOLO] 按 Ctrl+C 停止\n")
        
        try:
            while self.running:
                current_time = time.time()
                
                # 检测是否该进行YOLO检测
                detection_interval = 1.0 / DETECTION_FREQUENCY
                if current_time - self.last_detection_time >= detection_interval:
                    try:
                        # 动态更新窗口位置
                        if current_time - self.last_position_update_time >= WINDOW_POSITION_UPDATE_INTERVAL:
                            new_monitor = get_window_position()
                            if new_monitor != self.monitor:
                                self.monitor = new_monitor
                                print(f"[YOLO] 窗口位置已更新: {self.monitor}")
                            self.last_position_update_time = current_time
                        
                        # 进行YOLO检测
                        detections = self.detect_once()
                        
                        # 更新计数器
                        self.detection_count += 1
                        self.last_detection_time = current_time

                        # 日志输出
                        if detections:
                            print(f"[YOLO] #{self.detection_count} 检测到 {len(detections)} 个敌人 (置信度>{CONFIDENCE_THRESHOLD}): {detections}")
                        elif self.detection_count % 20 == 0:  # 每20次检测输出一次状态
                            print(f"[YOLO] #{self.detection_count} 未检测到敌人 (窗口位置: {self.monitor['left']},{self.monitor['top']})")

                        # 发布检测结果
                        self.publish_detection(detections)

                        # 显示检测结果（可选且需要启用）
                        if SHOW_DETECTION_WINDOW and ROS2_AVAILABLE:  # 仅在有ROS2环境时显示GUI
                            try:
                                # 重新获取检测结果用于可视化
                                img = np.array(self.sct.grab(self.monitor))[:, :, :3]
                                results = self.model.predict(source=img, conf=CONFIDENCE_THRESHOLD, verbose=False, device="cpu")[0]
                                annotated_img = results.plot()
                                cv2.imshow("YOLO Detection", annotated_img)
                                cv2.waitKey(1)
                            except Exception as e:
                                pass  # 忽略可视化错误

                    except Exception as e:
                        self.error_count += 1
                        print(f"[YOLO] 检测错误 #{self.error_count}: {e}")
                        self.last_detection_time = current_time  # 防止错误堆积
                        
                        # 错误过多时尝试重新获取窗口位置
                        if self.error_count >= 5:
                            print("[YOLO] 错误过多，尝试重新获取窗口位置")
                            self.monitor = get_window_position()
                            self.error_count = 0

                # ROS2 回调处理
                if self.node and ROS2_AVAILABLE:
                    try:
                        rclpy.spin_once(self.node, timeout_sec=0.01)
                    except:
                        pass
                
                time.sleep(0.01)

        except KeyboardInterrupt:
            print(f"\n[YOLO] 用户停止 - 总共进行了 {self.detection_count} 次检测")
        except Exception as e:
            print(f"\n[YOLO] 发生未预期错误: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        print("[YOLO] 正在清理资源...")
        try:
            if ROS2_AVAILABLE:
                cv2.destroyAllWindows()
            if self.node:
                self.node.destroy_node()
            if ROS2_AVAILABLE and rclpy.ok():
                rclpy.shutdown()
            print("[YOLO] 资源清理完成")
        except Exception as e:
            print(f"[YOLO] 清理过程中发生错误: {e}")

def main():
    """主函数"""
    print("YOLO 敌人检测发布器 - 集成版本")
    print("=" * 50)
    
    # 检查关键依赖
    if not YOLO_AVAILABLE:
        print("❌ YOLO不可用，无法运行")
        return
    
    if not ROS2_AVAILABLE:
        print("⚠️  ROS2不可用，将使用模拟模式")
    
    # 创建并运行发布器
    publisher = YOLOEnemyPublisher()
    publisher.run()

if __name__ == "__main__":
    main()