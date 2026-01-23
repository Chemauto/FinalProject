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
WINDOW_POSITION_UPDATE_INTERVAL = 10.0  # 窗口位置更新间隔（秒）
MAX_RETRY_ATTEMPTS = 3  # 最大重试次数

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
                    
                    # 验证窗口尺寸（允许一定误差）
                    if abs(w - 800) <= 50 and abs(h - 600) <= 50:
                        geometry = {"left": x, "top": y, "width": w, "height": h}
                        print(f"[YOLO] 使用simulator_position检测到窗口: {geometry}")
                        return geometry
                    else:
                        print(f"[YOLO] 窗口尺寸接近: 实际{w}x{h}，期望800x600")
                        # 对于接近的尺寸，我们也接受
                        geometry = {"left": x, "top": y, "width": w, "height": h}
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
    
    def detect_once(self):
        """进行一次检测"""
        if not self.model or not self.sct:
            return []
        
        try:
            # 进行YOLO检测
            img = np.array(self.sct.grab(self.monitor))[:, :, :3]
            results = self.model.predict(source=img, conf=CONFIDENCE_THRESHOLD, verbose=False, device="cpu")[0]

            # 转换检测结果
            detections = []
            if results.boxes is not None:
                for i, box in enumerate(results.boxes):
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    center_x = float((x1 + x2) / 2) + self.monitor["left"]
                    center_y = float((y1 + y2) / 2) + self.monitor["top"]
                    confidence = float(box.conf[0].cpu().numpy())
                    detections.append({
                        "id": f"yolo_{i}", 
                        "x": round(center_x, 2), 
                        "y": round(center_y, 2),
                        "confidence": round(confidence, 3)
                    })
            
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

                        # 显示检测结果（可选）
                        if ROS2_AVAILABLE:  # 仅在有ROS2环境时显示GUI
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