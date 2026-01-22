# YOLO 目标检测模块

## 功能概述

提供多种目标检测方式，用于识别敌人位置：

1. **鼠标点击检测** - 在仿真器上直接点击标记敌人
2. **键盘输入检测** - 通过坐标输入设置敌人位置
3. **YOLO 视觉识别** - 使用 YOLO 模型自动检测（接口已预留）

## 快速开始

### 方式1: 鼠标点击（推荐）

```bash
# 启动仿真器
python Test_Module/enhanced_simulator.py

# 按 'M' 进入鼠标标记模式
# 点击地图生成敌人
# 再次按 'M' 退出
```

### 方式2: 键盘输入

```bash
# 运行 Demo
python Yolo_Module/demo.py

# 选择 1. 键盘输入坐标
# 输入: 700,300
```

### 方式3: 代码集成

```python
import asyncio
from Yolo_Module.target_detector import create_detector, DetectionMethod

async def main():
    detector = create_detector(DetectionMethod.KEYBOARD)
    targets = await detector.detect()

    for target in targets:
        print(f"目标 {target.id}: ({target.x}, {target.y})")

asyncio.run(main())
```

## 目录结构

```
Yolo_Module/
├── __init__.py              # 模块初始化
├── target_detector.py       # 目标检测器（核心）
├── coordinate_mapper.py     # 坐标映射工具
├── screen_capture.py        # 屏幕截图工具
├── demo.py                  # Demo 演示程序
└── README.md                # 本文件
```

## API 文档

### 创建检测器

```python
from Yolo_Module.target_detector import create_detector, DetectionMethod

# 键盘输入检测
detector = create_detector(DetectionMethod.KEYBOARD)

# 鼠标点击检测
detector = create_detector(DetectionMethod.MOUSE)

# YOLO 检测（未来）
detector = create_detector(DetectionMethod.YOLO, model_path="yolov8n.pt")
```

### 检测目标

```python
# 检测
targets = await detector.detect()

# 获取结果
for target in targets:
    print(f"ID: {target.id}")
    print(f"位置: ({target.x}, {target.y})")
    print(f"置信度: {target.confidence}")
    print(f"方法: {target.method}")

# 格式化为字典
results = detector.format_results()
```

### 坐标映射

```python
from Yolo_Module.coordinate_mapper import CoordinateMapper

mapper = CoordinateMapper(window_offset=(8, 30))

# 屏幕 -> 仿真
sim_x, sim_y = mapper.screen_to_sim(708, 330)
# 结果: (700, 300)

# 仿真 -> 屏幕
screen_x, screen_y = mapper.sim_to_screen(700, 300)
# 结果: (708, 330)
```

## 仿真器集成

在 `Test_Module/enhanced_simulator.py` 中已集成鼠标点击功能：

```
按键操作:
  [M] - 进入/退出鼠标标记模式
  点击左键 - 在点击位置生成敌人
```

## 运行 Demo

```bash
python Yolo_Module/demo.py
```

Demo 包含：
1. 键盘输入坐标生成敌人
2. 坐标映射测试
3. 使用说明

## 依赖

### 核心功能
- 无需额外依赖

### 可选功能
```bash
# 屏幕截图
pip install pillow pywin32

# YOLO 检测（未来）
pip install ultralytics
```

## 扩展开发

### 添加自定义检测器

```python
from Yolo_Module.target_detector import TargetDetector, DetectedTarget

class MyDetector(TargetDetector):
    async def detect(self):
        # 实现检测逻辑
        return [DetectedTarget(
            id="1",
            x=100,
            y=200,
            confidence=0.95,
            class_name="custom",
            method="my_method"
        )]
```

## 技术细节

### 坐标系统

- **屏幕坐标**: 相对于窗口左上角（包含边框和标题栏）
- **仿真坐标**: Pygame 内部坐标（800x600）

### 映射精度

- 坐标转换精度: ±0 像素
- 鼠标响应时间: ~16ms (60fps)

## 未来计划

- [ ] 实现 YOLO 检测器
- [ ] 训练自定义 YOLO 模型
- [ ] 支持视频流检测
- [ ] 实时检测循环

---

**版本**: 1.0.0
**状态**: ✅ 核心功能已完成
