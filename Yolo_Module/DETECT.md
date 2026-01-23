# YOLO 实时检测 - 截屏版

## 快速使用

```bash
# 终端1：启动仿真器（确保窗口在屏幕左上角）
python3 Sim_Module/sim2d/simulator.py

# 终端2：YOLO实时检测（自动截屏）
python3 Yolo_Module/yolo_detector.py
```

## 工作原理

- **自动截屏**：直接截取屏幕左上角800x600区域
- **实时检测**：YOLO模型检测截图中的敌人
- **显示结果**：在窗口中实时显示检测框

## 调整检测区域

如果仿真器窗口不在左上角，修改 `yolo_detector.py` 中的 `MONITOR`：

```python
# 仿真器窗口位置
MONITOR = {"top": 100,    # 窗口距离顶部的像素
           "left": 200,   # 窗口距离左侧的像素
           "width": 800,  # 窗口宽度
           "height": 600} # 窗口高度
```

## 配置

编辑 `yolo_detector.py`：
- `MODEL` - 模型路径
- `MONITOR` - 检测区域
- `conf=0.5` - 置信度阈值

## 操作

- 按 `q` 键退出检测
- 确保仿真器窗口完全可见
- 代码行数：37行
