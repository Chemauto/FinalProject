# Yolo_Module - YOLO 目标检测模块

基于 YOLO 的敌人检测模块，用于替代仿真器直接获取坐标的方式。

## 功能

1. **自动生成训练数据** - 随机撒点截图并生成 YOLO 格式标注
2. **标注可视化** - 验证标注是否正确
3. **YOLO 检测** - 使用训练好的模型进行目标检测
4. **ROS 话题发布** - 将检测结果发布到 `/robot/yolo_enemies` 话题
5. **自动窗口位置检测** - 使用 `xwininfo` 自动获取仿真器窗口位置，无需手动配置

## 目录结构

```
Yolo_Module/
├── README.md                  # 本文档
├── best.pt                    # 训练好的 YOLO 模型
├── data/                      # 数据目录
│   ├── images/               # 原始截图
│   ├── labels/               # YOLO 标注文件
│   ├── bbox_viz/             # 带边界框的可视化图片
│   └── metadata.json         # 数据集元数据
├── yolo_simulator.py         # 训练数据生成器
├── visualize_labels.py       # 标注可视化工具
├── yolo_detector.py          # YOLO 检测器（手动刷新）
├── yolo_publisher.py         # ROS 发布器（实时检测）
└── simulator_position.py     # 窗口位置检测工具
```

## 快速开始

### 数据集准备（两种方式）

#### 方式1：使用预生成的1000张数据集（推荐）

已生成的数据集位于：`/home/xcj/work/testyolo/YoloTest/datasets/sim_enemy`

**数据集规格：**
- 总样本数：1000张
- 训练集：900张（`images/train/`）
- 验证集：100张（`images/val/`）
- 图片尺寸：800 x 600 像素
- 敌人类型：圆形，半径15像素，红色
- 标注格式：YOLO格式（归一化坐标）

**使用方法：**

训练时使用该数据集的YAML配置文件：

```bash
from ultralytics import YOLO

# 加载预训练模型
model = YOLO('yolov8n.pt')  # 或 yolov11n.pt

# 训练（使用外部数据集配置）
model.train(
    data='/home/xcj/work/testyolo/YoloTest/datasets/sim_enemy/sim_enemy.yaml',
    epochs=100,
    imgsz=640,
    batch=16
)
```

**数据集目录结构：**
```
/home/xcj/work/testyolo/YoloTest/datasets/sim_enemy/
├── sim_enemy.yaml          # 数据集配置文件
├── images/
│   ├── train/              # 900张训练图片
│   │   ├── sample_00000.png
│   │   └── ...
│   └── val/                # 100张验证图片
│       ├── sample_00900.png
│       └── ...
└── labels/
    ├── train/              # 900个训练标注
    │   ├── sample_00000.txt
    │   └── ...
    └── val/                # 100个验证标注
        ├── sample_00900.txt
        └── ...
```

#### 方式2：生成自定义训练数据

```bash
cd /home/xcj/work/FinalProject
python3 Yolo_Module/yolo_simulator.py
```

这将生成：
- `data/images/` - 默认100张仿真器截图（可修改）
- `data/labels/` - 对应的 YOLO 格式标注文件
- `data/bbox_viz/` - 带边界框的可视化图片
- `data/metadata.json` - 数据集元数据

**自定义生成参数：**

编辑 `yolo_simulator.py` 中的配置：
```python
OUTPUT_DIR = "data"         # 输出目录
NUM_SAMPLES = 1000          # 生成样本数
MIN_ENEMIES = 1             # 最少敌人数
MAX_ENEMIES = 5             # 最多敌人数
```

**手动分割训练集和验证集：**

```bash
# 将生成的前900张作为训练集，后100张作为验证集
mkdir -p data/images/train data/images/val
mkdir -p data/labels/train data/labels/val

# 移动文件（示例脚本）
mv data/images/sample_00[0-8]*.png data/images/train/
mv data/images/sample_009*.png data/images/val/
mv data/labels/sample_00[0-8]*.txt data/labels/train/
mv data/labels/sample_009*.txt data/labels/val/
```

然后修改 `data.yaml.template` 中的路径：
```yaml
path: /home/xcj/work/FinalProject/Yolo_Module/data
train: images/train
val: images/val
```

### 2. 验证标注

```bash
# 可视化单张图片
python3 Yolo_Module/visualize_labels.py --single --show

# 批量可视化（不显示窗口）
python3 Yolo_Module/visualize_labels.py
```

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

**训练命令（使用外部1000张数据集）：**
```bash
from ultralytics import YOLO

# 加载预训练模型
model = YOLO('yolov8n.pt')  # 或 yolov11n.pt

# 训练（使用外部数据集）
model.train(
    data='/home/xcj/work/testyolo/YoloTest/datasets/sim_enemy/sim_enemy.yaml',
    epochs=100,
    imgsz=640,
    batch=16
)

# 模型保存在 runs/detect/train/weights/best.pt
```

**或使用本地生成的数据集：**
```bash
from ultralytics import YOLO

# 加载预训练模型
model = YOLO('yolov8n.pt')

# 训练（使用本地数据集，需先复制data.yaml.template为data.yaml）
model.train(
    data='Yolo_Module/data.yaml',
    epochs=100,
    imgsz=640,
    batch=16
)
```

### 4. 使用训练好的模型检测

**方式1：手动刷新检测（适合调试）**

```bash
# 启动仿真器（终端1）
python3 Sim_Module/sim2d/simulator.py

# 启动 YOLO 检测器（终端2）- 自动检测窗口位置
python3 Yolo_Module/yolo_detector.py
```

操作方式：
- 程序启动时自动使用 `xwininfo` 检测仿真器窗口位置
- 按任意键刷新检测
- 按 `q` 键退出

**方式2：实时检测（推荐）**

```bash
# 启动仿真器（终端1）
python3 Sim_Module/sim2d/simulator.py

# 启动 YOLO 发布器（终端2）- 自动检测窗口位置
python3 Yolo_Module/yolo_publisher.py --rate 1.0
```

可选参数：
```bash
# 指定模型路径
python3 Yolo_Module/yolo_publisher.py --model best.pt

# 调整置信度阈值（默认0.85）
python3 Yolo_Module/yolo_publisher.py --conf 0.7

# 调整发布频率（默认1Hz）
python3 Yolo_Module/yolo_publisher.py --rate 2.0

# 不显示检测窗口（后台运行）
python3 Yolo_Module/yolo_publisher.py --no-show
```

**窗口位置自动检测说明：**
- `yolo_detector.py` 和 `yolo_publisher.py` 启动时自动调用 `xwininfo` 获取仿真器窗口位置
- 无需手动设置 `MONITOR` 坐标
- 支持窗口在屏幕任意位置
- 如果检测失败，使用默认位置 (0, 0)

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
OUTPUT_DIR = "/home/xcj/work/testyolo/YoloTest/datasets/sim_enemy"
NUM_SAMPLES = 1000          # 生成样本数
TRAIN_SPLIT = 0.9           # 训练集比例（900 train + 100 val）
MIN_ENEMIES = 1             # 最少敌人数
MAX_ENEMIES = 5             # 最多敌人数
```

**修改参数：**编辑脚本顶部的常量。

**生成的数据集结构：**
```
sim_enemy/
├── sim_enemy.yaml          # 数据集配置（自动生成）
├── images/
│   ├── train/              # 训练图片
│   └── val/                # 验证图片
└── labels/
    ├── train/              # 训练标注
    └── val/                # 验证标注
```

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

YOLO 检测器 - 手动刷新版（适合调试）。

**功能：**
- 启动时自动使用 `xwininfo` 检测仿真器窗口位置
- 按任意键刷新检测
- 按 `q` 键退出

**使用方式：**
```bash
# 启动仿真器后运行
python3 Yolo_Module/yolo_detector.py
```

**窗口检测逻辑：**
```python
def get_simulator_position():
    """使用 xwininfo 获取仿真器窗口位置"""
    result = subprocess.run(
        ['xwininfo', '-name', '追击功能测试 - 2D Robot Simulator'],
        capture_output=True, text=True, timeout=5
    )
    # 解析输出获取 top, left, width, height
```

### yolo_publisher.py

ROS 发布器 - 实时检测（推荐使用）。

**功能：**
- 启动时自动使用 `xwininfo` 检测仿真器窗口位置
- 实时截屏并使用 YOLO 检测敌人
- 发布检测结果到 `/robot/yolo_enemies` 话题
- 显示检测窗口（可选）

**参数：**
- `--model`: YOLO 模型路径（默认：`best.pt`）
- `--conf`: 置信度阈值（默认：0.85）
- `--rate`: 发布频率 Hz（默认：1.0）
- `--duration`: 运行时长秒（默认：无限）
- `--no-show`: 不显示检测窗口

**示例：**
```bash
# 使用默认参数（推荐）
python3 Yolo_Module/yolo_publisher.py

# 调整置信度和频率
python3 Yolo_Module/yolo_publisher.py --conf 0.7 --rate 2.0

# 后台运行（不显示窗口）
python3 Yolo_Module/yolo_publisher.py --no-show --rate 1.0

# 运行指定时长
python3 Yolo_Module/yolo_publisher.py --duration 60
```

**输出示例：**
```
[YoloPublisher] ✓ 自动检测到仿真器窗口位置: {'top': 49, 'left': 14, 'width': 800, 'height': 600}
[YoloPublisher] 已发布 2 个敌人位置: [{'id': 'yolo_0', 'x': 400.0, 'y': 300.0}, ...]
```

### simulator_position.py

窗口位置检测工具（独立使用）。

**功能：**
- 使用 `xwininfo` 获取仿真器窗口位置
- 支持单次检测和实时测试模式

**使用方式：**
```bash
# 单次检测（输出坐标）
python3 Yolo_Module/simulator_position.py
# 输出: 14,49

# 测试模式（显示详细信息）
python3 Yolo_Module/simulator_position.py --test

# 显示帮助
python3 Yolo_Module/simulator_position.py --help
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
2. 置信度阈值过高（尝试降低到 0.5 或 0.3）
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

### Q: 窗口位置检测失败？

A: 解决方法：
1. 确保仿真器正在运行
2. 确保仿真器窗口标题为 "追击功能测试 - 2D Robot Simulator"
3. 检查是否安装了 `xwininfo` 工具：`which xwininfo`
4. 如果检测失败，程序会使用默认位置 (0, 0)，手动将窗口拖到左上角即可

### Q: 检测区域不正确？

A: 检查步骤：
1. 运行 `python3 Yolo_Module/simulator_position.py --test` 查看窗口位置
2. 启动 `yolo_detector.py` 或 `yolo_publisher.py` 时查看输出的窗口位置
3. 确保检测到的窗口尺寸是 800x600
4. 如果窗口移动了，重新启动检测程序即可自动获取新位置

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

### 使用预生成数据集（推荐）

1. **验证数据集**：检查 `/home/xcj/work/testyolo/YoloTest/datasets/sim_enemy/` 中的图片和标注
2. **训练模型**：使用 `sim_enemy.yaml` 配置文件训练
3. **测试检测**：`python3 Yolo_Module/yolo_detector.py`
4. **集成到系统**：修改 Robot_Module 使用 YOLO 检测

### 使用自生成数据集

1. **生成数据**：`python3 Yolo_Module/yolo_simulator.py`（修改NUM_SAMPLES=1000）
2. **分割数据集**：手动将900张放入train/，100张放入val/
3. **检查标注**：`python3 Yolo_Module/visualize_labels.py`
4. **训练模型**：使用 ultralytics 训练
5. **测试检测**：`python3 Yolo_Module/yolo_detector.py`
6. **集成到系统**：修改 Robot_Module 使用 YOLO 检测

---

**训练愉快！** 🎯
