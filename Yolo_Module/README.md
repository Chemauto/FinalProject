# Yolo_Module - YOLO 目标检测模块

基于 YOLOv8 的目标检测模块，支持屏幕截图、目标检测和坐标映射。

## 📁 模块结构

```
Yolo_Module/
├── README.md                  # 本文档
├── yolo_simulator.py          # YOLO 追击仿真器
└── test/                      # YOLO 核心模块
    ├── __init__.py
    ├── yolo_detector.py       # YOLO 检测器
    ├── screen_capture.py      # 屏幕截图工具
    └── coordinate_mapper.py   # 坐标映射器
```

## 🎯 功能特性

- **YOLO 目标检测**: 使用 ultralytics YOLOv8 检测图像中的目标
- **屏幕截图**: 支持全屏截图和区域截图（pyautogui 或 mss）
- **坐标映射**: 屏幕坐标 ↔ 仿真器坐标双向映射
- **模块化设计**: 独立的模块，易于集成和扩展
- **延迟初始化**: YOLO 模型按需加载，减少启动时间

## 🔧 核心组件

### test/yolo_detector.py - YOLO 检测器

**YoloDetector 类：**

```python
from Yolo_Module.test import YoloDetector

# 初始化检测器
detector = YoloDetector(model_name="yolov8n.pt")

# 检测目标
detections = detector.detect(image_array, conf_threshold=0.25)

# 检测结果格式
# [{
#   "center": (x, y),        # 中心坐标
#   "confidence": 0.85,      # 置信度
#   "class": "person",       # 类别
#   "box": (x1, y1, x2, y2)  # 边界框
# }]

# 只检测人员
person_detections = detector.detect_person(image_array)
```

**支持的模型：**
- `yolov8n.pt` - Nano (最快，推荐实时应用)
- `yolov8s.pt` - Small
- `yolov8m.pt` - Medium
- `yolov8l.pt` - Large
- `yolov8x.pt` - XLarge (最准确)

### test/screen_capture.py - 屏幕截图工具

**ScreenCapture 类：**

```python
from Yolo_Module.test import ScreenCapture

# 初始化截图工具
capturer = ScreenCapture()

# 截取全屏
image = capturer.capture_screen()

# 截取区域
image = capturer.capture_region(x=100, y=100, width=800, height=600)

# 保存图像
ScreenCapture.save_image(image_array, "/tmp/screenshot.jpg")

# 保存图像（带时间戳）
filename = ScreenCapture.save_image_with_timestamp(
    image_array,
    directory="/tmp",
    prefix="capture"
)
```

**依赖项：**
- `pyautogui` 或 `mss` - 屏幕截图
- `opencv-python` - 图像保存

### test/coordinate_mapper.py - 坐标映射器

**CoordinateMapper 类：**

```python
from Yolo_Module.test import CoordinateMapper

# 初始化映射器
mapper = CoordinateMapper(
    screen_region=(100, 100, 800, 600),  # x, y, width, height
    simulator_size=(800, 600)            # width, height
)

# 屏幕坐标 → 仿真器坐标
sim_x, sim_y = mapper.screen_to_simulator(screen_x=400, screen_y=300)

# 仿真器坐标 → 屏幕坐标
screen_x, screen_y = mapper.simulator_to_screen(sim_x=400, sim_y=300)

# 绝对屏幕坐标 → 仿真器坐标
sim_x, sim_y = mapper.absolute_screen_to_simulator(abs_screen_x=500, abs_screen_y=400)

# 动态更新配置
mapper.set_screen_region((200, 200, 800, 600))
mapper.set_simulator_size((1024, 768))
```

### yolo_simulator.py - YOLO 追击仿真器

**YoloSimulator 类：**

集成 YOLO 检测、屏幕截图和 2D 仿真器的完整仿真系统。

```python
from Yolo_Module.yolo_simulator import YoloSimulator

# 启动仿真器
simulator = YoloSimulator()
simulator.run()
```

**操作说明：**
- **鼠标左键拖动**: 选择区域进行截图和检测
- **按 S**: 设置屏幕截图区域
- **按 C**: 清除所有敌人
- **按 L**: 切换追击线显示
- **按 ESC**: 退出

**工作流程：**
1. 启动仿真器：`python3 Yolo_Module/yolo_simulator.py`
2. 按 `S` 设置屏幕截图区域
3. 在仿真器中拖动鼠标选择区域
4. 自动使用 YOLO 检测并生成敌人
5. 通过 ROS2 接口执行追击

## 🚀 快速开始

### 1. 安装依赖

```bash
# 基础依赖
pip install ultralytics opencv-python numpy

# 屏幕截图（任选其一）
pip install pyautogui
# 或
pip install mss

# Pygame（仿真器）
pip install pygame
```

### 2. 下载 YOLO 模型

首次运行时，YOLO 会自动下载模型。也可以手动下载：

```bash
# 使用 YOLO CLI
yolo detect model=yolov8n.pt

# 或直接下载
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
```

### 3. 运行仿真器

```bash
# 启动 YOLO 追击仿真器
python3 Yolo_Module/yolo_simulator.py

# 按 S 设置屏幕区域，格式: x y width height
# 示例: 100 100 800 600
```

### 4. 测试单个模块

```bash
# 测试 YOLO 检测器
python3 Yolo_Module/test/yolo_detector.py

# 测试屏幕截图
python3 Yolo_Module/test/screen_capture.py

# 测试坐标映射
python3 Yolo_Module/test/coordinate_mapper.py
```

## 💡 使用场景

### 场景 1: 检测图像中的目标

```python
from Yolo_Module.test import YoloDetector, ScreenCapture
import cv2

# 初始化
detector = YoloDetector(model_name="yolov8n.pt")
capturer = ScreenCapture()

# 截图
image = capturer.capture_region(100, 100, 800, 600)

# 检测
detections = detector.detect(image)

# 保存结果（绘制边界框）
for det in detections:
    x1, y1, x2, y2 = det["box"]
    cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
    cv2.putText(image, det["class"], (int(x1), int(y1) - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

cv2.imwrite("/tmp/result.jpg", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
```

### 场景 2: 集成到仿真器

```python
from Yolo_Module.test import YoloDetector, ScreenCapture, CoordinateMapper
from Yolo_Module.yolo_simulator import YoloSimulator

# 创建仿真器
simulator = YoloSimulator()

# 仿真器会自动：
# 1. 监听鼠标拖动选择区域
# 2. 截取选定区域
# 3. 使用 YOLO 检测
# 4. 在检测到的位置生成敌人
# 5. 通过 ROS2 发布追击命令

simulator.run()
```

### 场景 3: 与 ROS2 集成

```python
from Yolo_Module.test import YoloDetector, ScreenCapture, CoordinateMapper
from ros_topic_comm import get_shared_queue

# 初始化
detector = YoloDetector()
capturer = ScreenCapture()
mapper = CoordinateMapper(screen_region=(100, 100, 800, 600),
                         simulator_size=(800, 600))

# 获取 ROS 队列
queue = get_shared_queue()

# 截图并检测
image = capturer.capture_region(100, 100, 800, 600)
detections = detector.detect(image)

# 发布追击命令
for det in detections:
    screen_x, screen_y = det["center"]
    sim_x, sim_y = mapper.screen_to_simulator(screen_x, screen_y)

    # 发布敌人位置
    from ros_topic_comm import set_enemy_positions
    set_enemy_positions([{"id": 0, "x": sim_x, "y": sim_y}])

    # 发布追击命令
    queue.put({
        "action": "chase_enemy",
        "parameters": {}
    })
```

## 🐛 调试技巧

### 1. 查看检测结果

```python
detections = detector.detect(image)
for det in detections:
    print(f"类别: {det['class']}")
    print(f"置信度: {det['confidence']:.2f}")
    print(f"中心: ({det['center'][0]:.1f}, {det['center'][1]:.1f})")
    print(f"边界框: {det['box']}")
```

### 2. 保存截图调试

```python
# 仿真器会自动保存截图到 /tmp/yolo_capture_*.jpg
# 查看截图确认区域是否正确
ls -l /tmp/yolo_capture_*.jpg
```

### 3. 调整 YOLO 参数

```python
# 提高置信度阈值（减少误检）
detections = detector.detect(image, conf_threshold=0.5)

# 使用更大的模型（提高准确率）
detector = YoloDetector(model_name="yolov8m.pt")
```

### 4. 测试坐标映射

```python
# 测试屏幕坐标到仿真器坐标的映射
mapper = CoordinateMapper(screen_region=(100, 100, 800, 600),
                         simulator_size=(800, 600))

# 测试关键点
test_points = [
    (0, 0),           # 左上角
    (400, 300),       # 中心
    (800, 600),       # 右下角
]

for sx, sy in test_points:
    sim_x, sim_y = mapper.screen_to_simulator(sx, sy)
    print(f"屏幕({sx}, {sy}) → 仿真器({sim_x:.1f}, {sim_y:.1f})")
```

## 🔧 配置说明

### YOLO 模型选择

| 模型 | 大小 | 速度 | 准确率 | 推荐场景 |
|------|------|------|--------|----------|
| yolov8n | 6MB | 最快 | 一般 | 实时应用 |
| yolov8s | 23MB | 快 | 良好 | 平衡选择 |
| yolov8m | 52MB | 中等 | 高 | 高精度需求 |
| yolov8l | 84MB | 慢 | 很高 | 离线处理 |
| yolov8x | 119MB | 最慢 | 最高 | 最佳精度 |

### 屏幕截图方式

```python
# 方式 1: pyautogui（推荐）
pip install pyautogui

# 方式 2: mss（更快）
pip install mss

# ScreenCapture 会自动选择可用的方式
```

## 📝 依赖

```
ultralytics>=8.0.0  # YOLOv8
opencv-python>=4.8.0  # 图像处理
numpy>=1.24.0  # 数组运算
pyautogui>=0.9.0  # 屏幕截图（可选）
mss>=9.0.0  # 屏幕截图（可选）
pygame>=2.5.0  # 仿真器
```

## 🔗 相关模块

- `Test_Module/chase_simulator.py` - 简单的追击仿真器
- `Sim_Module/sim2d/simulator.py` - 主仿真器
- `ros_topic_comm.py` - ROS2 通讯模块

## 🎯 性能优化

- **本地运行**: YOLO 推理在本地执行，无需网络
- **GPU 加速**: 自动检测并使用 CUDA（如果可用）
- **模型缓存**: 首次加载后缓存模型，减少后续初始化时间
- **延迟初始化**: 模型按需加载，减少启动时间

---

**目标检测，智能感知！** 🎯
