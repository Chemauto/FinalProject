# Interactive_Module - 交互界面模块

用户交互界面的核心模块，提供 CLI 命令行交互功能，协调 LLM 和 Robot_Module。

## 📁 模块结构

```
Interactive_Module/
├── README.md              # 本文档
└── interactive.py         # CLI 交互主程序
```

## 🎯 功能特性

- **自然语言交互**: 支持中文指令输入

====================================
Yolo_Module 模块详细文档
====================================

# Yolo_Module - YOLO 目标检测模块

基于 YOLO 算法的目标检测模块，提供屏幕捕获、目标识别、坐标映射等功能。

## 📁 模块结构

```
Yolo_Module/
├── README.md                  # 本文档
├── __init__.py
├── coordinate_mapper.py       # 坐标映射
├── demo.py                    # 演示程序
├── screen_capture.py          # 屏幕捕获
├── target_detector.py         # 目标检测器
└── test/                      # 测试文件
```

## 🎯 功能特性

- **实时检测**: 基于 YOLO 的实时目标检测
- **屏幕捕获**: 自动捕获屏幕内容
- **坐标映射**: 屏幕坐标到仿真器坐标的转换
- **多目标支持**: 同时检测多个目标
- **可视化**: 绘制检测框和标签

## 🔧 核心组件

### target_detector.py - 目标检测器

**TargetDetector 类：**

```python
class TargetDetector:
    """YOLO 目标检测器"""

    def __init__(self, model_name="yolov8n.pt"):
        """
        初始化检测器

        Args:
            model_name: YOLO 模型名称
        """
        from ultralytics import YOLO
        self.model = YOLO(model_name)

    def detect(self, image):
        """
        检测图像中的目标

        Args:
            image: OpenCV 图像 (numpy array)

        Returns:
            检测结果列表
            [
                {"class": "person", "confidence": 0.95, "bbox": [x, y, w, h]},
                ...
            ]
        """

    def detect_screen(self, region=None):
        """
        检测屏幕区域中的目标

        Args:
            region: 截图区域 (left, top, right, bottom)

        Returns:
            检测结果列表
        """

    def draw_detections(self, image, detections):
        """
        在图像上绘制检测结果

        Args:
            image: OpenCV 图像
            detections: 检测结果列表

        Returns:
            绘制后的图像
        """
```

**使用示例：**

```python
from Yolo_Module.target_detector import TargetDetector

# 初始化检测器
detector = TargetDetector(model_name="yolov8n.pt")

# 检测图像
detections = detector.detect(image)

# 遍历结果
for det in detections:
    print(f"类别: {det['class']}, 置信度: {det['confidence']:.2f}")
    print(f"位置: {det['bbox']}")

# 检测屏幕
detections = detector.detect_screen(region=(0, 0, 800, 600))
```

**支持的 YOLO 模型：**

- `yolov8n.pt` - YOLOv8 Nano（最快，精度一般）
- `yolov8s.pt` - YOLOv8 Small（推荐）
- `yolov8m.pt` - YOLOv8 Medium（平衡）
- `yolov8l.pt` - YOLOv8 Large（高精度，慢）
- `yolov8x.pt` - YOLOv8 XL（最高精度，最慢）

### screen_capture.py - 屏幕捕获

**ScreenCapture 类：**

```python
class ScreenCapture:
    """屏幕捕获工具"""

    def __init__(self):
        """初始化屏幕捕获"""

    def capture_screen(self, region=None):
        """
        捕获屏幕内容

        Args:
            region: 截图区域 (left, top, right, bottom)

        Returns:
            OpenCV 图像 (numpy array)
        """

    def capture_window(self, window_title):
        """
        捕获指定窗口的内容

        Args:
            window_title: 窗口标题

        Returns:
            OpenCV 图像
        """
```

**使用示例：**

```python
from Yolo_Module.screen_capture import ScreenCapture

# 初始化
capture = ScreenCapture()

# 捕获全屏
image = capture.capture_screen()

# 捕获指定区域
image = capture.capture_screen(region=(0, 0, 800, 600))

# 捕获指定窗口
image = capture.capture_window("2D Robot Simulator")
```

### coordinate_mapper.py - 坐标映射

**CoordinateMapper 类：**

```python
class CoordinateMapper:
    """坐标映射工具"""

    def __init__(self, screen_region, simulator_size):
        """
        初始化坐标映射器

        Args:
            screen_region: 屏幕区域 (left, top, right, bottom)
            simulator_size: 仿真器尺寸 (width, height)
        """

    def screen_to_simulator(self, x, y):
        """
        屏幕坐标转仿真器坐标

        Args:
            x, y: 屏幕坐标

        Returns:
            (sim_x, sim_y) 仿真器坐标
        """

    def simulator_to_screen(self, x, y):
        """
        仿真器坐标转屏幕坐标

        Args:
            x, y: 仿真器坐标

        Returns:
            (screen_x, screen_y) 屏幕坐标
        """
```

**使用示例：**

```python
from Yolo_Module.coordinate_mapper import CoordinateMapper

# 初始化（仿真器窗口在屏幕的 (100, 100) 到 (900, 700)）
mapper = CoordinateMapper(
    screen_region=(100, 100, 900, 700),
    simulator_size=(800, 600)
)

# 屏幕坐标转仿真器坐标
sim_x, sim_y = mapper.screen_to_simulator(500, 400)

# 仿真器坐标转屏幕坐标
screen_x, screen_y = mapper.simulator_to_screen(400, 300)
```

### demo.py - 演示程序

**功能：**

1. 捕获仿真器屏幕
2. 使用 YOLO 检测目标
3. 映射坐标
4. 绘制检测结果
5. 显示可视化窗口

**运行演示：**

```bash
python3 Yolo_Module/demo.py
```

## 🚀 快速开始

### 1. 安装依赖

```bash
# 安装 YOLO
pip install ultralytics

# 安装屏幕捕获
pip install pillow pyautogui

# 安装 OpenCV
pip install opencv-python
```

### 2. 下载模型

```bash
# 自动下载（首次运行时）
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"

# 或手动下载
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
```

### 3. 运行演示

```bash
# 启动仿真器（终端1）
python3 Sim_Module/sim2d/simulator.py

# 运行 YOLO 演示（终端2）
python3 Yolo_Module/demo.py
```

### 4. 查看结果

演示程序会显示：
- 原始截图
- 检测框
- 类别标签
- 置信度分数

## 💡 使用场景

### 场景 1: 检测仿真器中的敌人

```python
from Yolo_Module.target_detector import TargetDetector
from Yolo_Module.screen_capture import ScreenCapture

# 初始化
detector = TargetDetector()
capture = ScreenCapture()

# 捕获仿真器窗口
image = capture.capture_window("2D Robot Simulator")

# 检测目标
detections = detector.detect(image)

# 过滤类别
enemies = [d for d in detections if d['class'] == 'person']
```

### 场景 2: 实时目标追踪

```python
import cv2
import time

while True:
    # 捕获屏幕
    image = capture.capture_screen(region=(0, 0, 800, 600))

    # 检测目标
    detections = detector.detect(image)

    # 绘制结果
    image = detector.draw_detections(image, detections)

    # 显示
    cv2.imshow("YOLO Detection", image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    time.sleep(0.1)  # 10 FPS

cv2.destroyAllWindows()
```

### 场景 3: 坐标映射与追击

```python
from Yolo_Module.coordinate_mapper import CoordinateMapper

# 初始化映射器
mapper = CoordinateMapper(
    screen_region=(100, 100, 900, 700),
    simulator_size=(800, 600)
)

# 检测目标
detections = detector.detect_screen(region=(100, 100, 900, 700))

# 映射坐标
for det in detections:
    x, y, w, h = det['bbox']
    center_x = x + w / 2
    center_y = y + h / 2

    # 转换为仿真器坐标
    sim_x, sim_y = mapper.screen_to_simulator(center_x, center_y)

    print(f"目标在仿真器坐标: ({sim_x:.1f}, {sim_y:.1f})")
```

## 🔧 配置说明

### YOLO 模型选择

| 模型 | 大小 | 速度 | 精度 | 推荐场景 |
|------|------|------|------|----------|
| yolov8n | 6MB | 最快 | 一般 | 实时检测 |
| yolov8s | 23MB | 快 | 良好 | 平衡选择 ✅ |
| yolov8m | 52MB | 中等 | 很好 | 高精度需求 |
| yolov8l | 84MB | 慢 | 优秀 | 离线分析 |
| yolov8x | 119MB | 最慢 | 最佳 | 研究用途 |

### 检测参数调整

```python
# 调整置信度阈值
detections = detector.detect(image, conf_threshold=0.5)

# 调整 IoU 阈值
detections = detector.detect(image, iou_threshold=0.5)

# 限制检测数量
detections = detector.detect(image, max_detections=10)
```

## 🐛 调试技巧

### 1. 测试屏幕捕获

```python
from Yolo_Module.screen_capture import ScreenCapture
import cv2

capture = ScreenCapture()
image = capture.capture_screen()
cv2.imwrite("test_capture.png", image)
print("截图已保存到 test_capture.png")
```

### 2. 测试坐标映射

```python
from Yolo_Module.coordinate_mapper import CoordinateMapper

mapper = CoordinateMapper(
    screen_region=(100, 100, 900, 700),
    simulator_size=(800, 600)
)

# 测试四个角
print(mapper.screen_to_simulator(100, 100))  # 应该返回 (0, 0)
print(mapper.screen_to_simulator(900, 700))  # 应该返回 (800, 600)
```

### 3. 可视化检测框

```python
detections = detector.detect(image)
image = detector.draw_detections(image, detections)
cv2.imshow("Detections", image)
cv2.waitKey(0)
```

### 4. 保存检测结果

```python
import json

results = {
    'timestamp': time.time(),
    'detections': detections
}

with open('detections.json', 'w') as f:
    json.dump(results, f, indent=2)
```

## 🔗 相关模块

- `Sim_Module/sim2d/simulator.py` - 2D 仿真器
- `Robot_Module/module/chase.py` - 追击功能
- `ros_topic_comm.py` - ROS2 通讯

## 📝 依赖

```
ultralytics>=8.0.0  # YOLO 模型
opencv-python>=4.8.0  # 图像处理
pillow>=10.0.0  # 图像 I/O
pyautogui>=0.9.0  # 屏幕捕获（可选）
numpy>=1.24.0  # 数值计算
```

## 🎯 性能优化

- **使用 GPU**: `device='cuda'` 或 `device='0'`
- **减小输入尺寸**: `imgsz=640` 改为 `imgsz=320`
- **使用轻量模型**: `yolov8n.pt` 而非 `yolov8x.pt`
- **限制类别**: `classes=[0]` 只检测人（COCO 数据集）

## 📊 COCO 数据集类别

```
0: person (人)
1: bicycle (自行车)
2: car (汽车)
3: motorcycle (摩托车)
...
16: dog (狗)
17: cat (猫)
...
```

完整类别列表：[COCO Dataset](https://cocodataset.org/#overview)

---

**目标检测，精准识别！** 🎯
