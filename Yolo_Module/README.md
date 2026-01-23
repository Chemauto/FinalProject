# Yolo_Module - YOLO 目标检测模块

基于 YOLO 的敌人检测模块，用于替代仿真器直接获取坐标的方式。

## 功能

1. **自动生成训练数据** - 随机撒点截图并生成 YOLO 格式标注
2. **标注可视化** - 验证标注是否正确
3. **YOLO 检测** - 使用训练好的模型进行目标检测
4. **ROS 话题发布** - 将检测结果发布到 `/robot/yolo_enemies` 话题

## 目录结构

```
Yolo_Module/
├── README.md                  # 本文档
├── data/                      # 数据目录
│   ├── images/               # 原始截图
│   ├── labels/               # YOLO 标注文件
│   ├── bbox_viz/             # 带边界框的可视化图片
│   └── metadata.json         # 数据集元数据
├── yolo_simulator.py         # 训练数据生成器
├── visualize_labels.py       # 标注可视化工具
├── yolo_detector.py          # YOLO 检测器
└── yolo_publisher.py         # ROS 发布器
```

## 快速开始

### 1. 生成训练数据

```bash
cd /home/robot/work/FinalProject
python3 Yolo_Module/yolo_simulator.py
```

这将生成：
- `data/images/` - 100 张仿真器截图
- `data/labels/` - 对应的 YOLO 格式标注文件
- `data/bbox_viz/` - 带边界框的可视化图片
- `data/metadata.json` - 数据集元数据

### 2. 验证标注

```bash
# 可视化单张图片
python3 Yolo_Module/visualize_labels.py --single --show

# 批量可视化（不显示窗口）
python3 Yolo_Module/visualize_labels.py
```

### 3. 训练 YOLO 模型

**安装依赖：**
```bash
pip install ultralytics
```

**训练命令：**
```bash
from ultralytics import YOLO

# 加载预训练模型
model = YOLO('yolov8n.pt')  # 或 yolov11n.pt

# 训练
model.train(
    data='Yolo_Module/data.yaml',  # 数据配置文件（需要创建）
    epochs=100,
    imgsz=640,
    batch=16
)

# 保存模型
# 模型会保存在 runs/detect/train/weights/best.pt
```

**创建 data.yaml：**
```yaml
path: /home/robot/work/FinalProject/Yolo_Module/data  # 数据集根目录
train: images  # 训练图片目录（相对于 path）
val: images    # 验证图片目录（暂时使用相同目录）

names:
  0: enemy  # 类别名称
```

### 4. 使用训练好的模型检测

```bash
# 启动仿真器（终端1）
python3 Sim_Module/sim2d/simulator.py

# 启动 YOLO 检测发布器（终端2）
python3 Yolo_Module/yolo_publisher.py --model runs/detect/train/weights/best.pt --rate 1.0
```

## YOLO 标注格式

YOLO 使用归一化的边界框坐标：

```
class_id center_x center_y width height
```

其中所有值都在 [0, 1] 范围内。

### 坐标转换

对于圆形敌人（半径 15 像素）：
- 边界框是外接正方形 (30x30 像素)
- `center_x = enemy.x / image_width`
- `center_y = enemy.y / image_height`
- `width = (2 * radius) / image_width = 30 / 800`
- `height = (2 * radius) / image_height = 30 / 600`

### 示例

```
0 0.500000 0.400000 0.037500 0.050000
0 0.750000 0.600000 0.037500 0.050000
```

## 脚本说明

### yolo_simulator.py

自动生成训练数据的脚本。

**配置参数：**
```python
OUTPUT_DIR = "data"         # 输出目录
NUM_SAMPLES = 100           # 生成样本数
MIN_ENEMIES = 1             # 最少敌人数
MAX_ENEMIES = 5             # 最多敌人数
```

**修改参数：**编辑 `main()` 函数中的变量。

### visualize_labels.py

可视化 YOLO 标注的工具。

**参数：**
- `--single`: 单张图片模式
- `--show`: 显示窗口
- `--images`: 图片目录
- `--labels`: 标注目录
- `--output`: 输出目录

**示例：**
```bash
# 批量可视化并保存
python3 Yolo_Module/visualize_labels.py --output data/visualized

# 单张可视化并显示
python3 Yolo_Module/visualize_labels.py --single --show \
    --images data/images/sample_00000.png \
    --labels data/labels/sample_00000.txt
```

### yolo_detector.py

YOLO 检测器类。

**使用方式：**
```python
from Yolo_Module.yolo_detector import YoloDetector

# 创建检测器
detector = YoloDetector(model_path="best.pt", conf_threshold=0.5)

# 从文件检测
detections = detector.detect_from_file("test.png")

# 从 Pygame 屏幕检测
detections = detector.detect_from_screenshot(screen)

# 结果格式
# [{"id": "yolo_0", "x": 400.0, "y": 300.0, "conf": 0.95}, ...]
```

### yolo_publisher.py

ROS 发布器，截取屏幕并发布检测结果。

**参数：**
- `--model`: YOLO 模型路径
- `--conf`: 置信度阈值（默认 0.5）
- `--rate`: 发布频率 Hz（默认 1.0）
- `--duration`: 运行时长秒（默认无限）

**示例：**
```bash
python3 Yolo_Module/yolo_publisher.py \
    --model best.pt \
    --conf 0.7 \
    --rate 2.0
```

## ROS 话题

### 发布的话题

| 话题名称 | 消息类型 | 用途 |
|---------|---------|------|
| `/robot/yolo_enemies` | String | YOLO 检测的敌人位置 |

### 消息格式

```json
[
  {"id": "yolo_0", "x": 400.0, "y": 300.0},
  {"id": "yolo_1", "x": 600.0, "y": 200.0}
]
```

## 集成到现有系统

### 修改 Robot_Module

在 `Robot_Module/module/chase.py` 中添加新的 MCP 工具：

```python
@mcp.tool()
def get_enemy_positions_by_yolo():
    """
    使用 YOLO 检测获取敌人位置

    Returns:
        JSON 字符串：敌人位置列表
    """
    # 订阅 /robot/yolo_enemies 话题
    # 返回检测结果
    ...
```

### 修改追击流程

1. 原有流程：仿真器发布 `/robot/enemies`
2. 新流程：YOLO 检测发布 `/robot/yolo_enemies`
3. 追击模块可以选择使用哪个数据源

## 常见问题

### Q: YOLO 检测不到敌人？

A: 可能的原因：
1. 模型未训练或训练不足
2. 置信度阈值过高
3. 训练数据与测试场景差异过大

### Q: 检测位置不准确？

A: 改进方法：
1. 增加训练数据量
2. 增加数据多样性（不同位置、数量的敌人）
3. 使用更大的模型（yolov8s, yolov8m）
4. 调整训练参数

### Q: 如何提高检测速度？

A: 优化方法：
1. 使用更小的模型（yolov8n, yolov8s）
2. 降低输入分辨率
3. 使用 GPU 加速
4. 降低发布频率

## 依赖

```
pygame>=2.5.0
ultralytics>=8.0.0
rclpy>=1.0.0
Pillow>=10.0.0
numpy>=1.24.0
```

## 安装

```bash
# 基础依赖
pip install -r /home/robot/work/FinalProject/requirements.txt

# YOLO 相关
pip install ultralytics

# 屏幕捕获（如果需要）
pip install python-xlib
```

## 下一步

1. 生成训练数据：`python3 Yolo_Module/yolo_simulator.py`
2. 检查标注质量：`python3 Yolo_Module/visualize_labels.py`
3. 训练模型：使用 ultralytics 训练
4. 测试检测：`python3 Yolo_Module/yolo_detector.py`
5. 集成到系统：修改 Robot_Module 使用 YOLO 检测

---

**训练愉快！** 🎯
