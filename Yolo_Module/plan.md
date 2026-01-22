# YOLO 目标检测模块实现计划

## 一、需求分析

### 当前状态
- ✅ 敌人坐标在代码中直接生成
- ✅ 仿真器支持显示敌人

### 新需求
1. **过渡方案**：支持鼠标点击/键盘输入设置敌人位置
2. **最终目标**：通过 YOLO 视觉识别自动检测敌人坐标
3. **系统集成**：与现有的追击模块无缝集成

### 技术挑战
1. 如何从仿真器截图进行 YOLO 检测
2. 如何将 YOLO 检测结果转换为敌人坐标
3. 如何实时更新敌人位置

## 二、系统架构设计

### 2.1 模块结构

```
Yolo_Module/
├── __init__.py
├── plan.md                    # 本文件
├── README.md                  # 使用说明
│
├── target_detector.py         # 目标检测核心
│   ├── TargetDetector         # 检测器基类
│   ├── MouseInputDetector     # 鼠标输入检测器
│   ├── KeyboardInputDetector  # 键盘输入检测器
│   └── YOLODetector           # YOLO 检测器（预留）
│
├── screen_capture.py          # 屏幕截图工具
│   ├── capture_simulator()    # 截取仿真器画面
│   └── save_screenshot()      # 保存截图
│
├── coordinate_mapper.py       # 坐标映射
│   ├── screen_to_sim()        # 屏幕坐标转仿真坐标
│   └── sim_to_screen()        # 仿真坐标转屏幕坐标
│
└── yolo_integration.py        # YOLO 集成
    ├── YOLOModel              # YOLO 模型包装
    ├── load_model()           # 加载 YOLO 模型
    └── detect_targets()       # 检测目标
```

### 2.2 数据流

```
┌─────────────────────────────────────────────────────┐
│                    用户输入                          │
│  ┌──────────────┐  ┌──────────────┐                │
│  │  鼠标点击    │  │  键盘输入    │                │
│  │  (过渡方案)  │  │  (过渡方案)  │                │
│  └──────┬───────┘  └──────┬───────┘                │
└─────────┼──────────────────┼────────────────────────┘
          │                  │
          ▼                  ▼
┌─────────────────────────────────────────────────────┐
│            TargetDetector (目标检测器)              │
│  • 统一的接口                                        │
│  • 支持多种输入方式                                  │
│  • 返回标准化的坐标数据                              │
└──────────────────────┬──────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────┐
│              EnemyManager (敌人管理器)              │
│  • 更新敌人位置                                      │
│  • 移除旧敌人                                        │
│  • 添加新敌人                                        │
└──────────────────────┬──────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────┐
│          ChaseController (追击控制器)               │
│  • 获取最新敌人位置                                  │
│  • 执行追击任务                                      │
└─────────────────────────────────────────────────────┘
```

### 2.3 YOLO 集成流程（未来）

```
仿真器运行 → 截图 → YOLO 检测 → 提取坐标 → 更新敌人 → 追击
    ↑                                                              │
    └────────────────── 循环检测 (每N秒) ←────────────────────────┘
```

## 三、核心接口设计

### 3.1 TargetDetector 基类

```python
class TargetDetector(ABC):
    """目标检测器基类"""

    @abstractmethod
    async def detect(self) -> List[Dict]:
        """
        检测目标

        Returns:
            [
                {
                    "id": "1",
                    "x": 100,
                    "y": 200,
                    "confidence": 0.95,
                    "class": "enemy"
                },
                ...
            ]
        """
        pass
```

### 3.2 MouseInputDetector

```python
class MouseInputDetector(TargetDetector):
    """鼠标输入检测器"""

    def __init__(self, simulator_window_title="2D Robot Simulator"):
        self.simulator_window = simulator_window_title

    async def detect(self):
        """等待用户点击，返回点击位置"""
        # 监听鼠标点击
        # 返回点击坐标
        pass
```

### 3.3 KeyboardInputDetector

```python
class KeyboardInputDetector(TargetDetector):
    """键盘输入检测器"""

    async def detect(self):
        """等待用户输入坐标"""
        x = input("请输入 X 坐标: ")
        y = input("请输入 Y 坐标: ")
        return [{"x": float(x), "y": float(y), "id": "1"}]
```

### 3.4 YOLODetector（预留）

```python
class YOLODetector(TargetDetector):
    """YOLO 目标检测器"""

    def __init__(self, model_path):
        self.model = YOLO(model_path)

    async def detect(self):
        """从截图检测目标"""
        screenshot = capture_simulator()
        results = self.model(screenshot)
        # 转换检测结果为坐标
        return coordinates
```

## 四、实现步骤

### Phase 1: 基础框架 ✅
- [x] 创建模块结构
- [x] 实现 TargetDetector 基类
- [x] 设计统一接口

### Phase 2: 过渡方案
- [ ] 实现 MouseInputDetector
- [ ] 实现 KeyboardInputDetector
- [ ] 集成到增强仿真器

### Phase 3: 屏幕截图
- [ ] 实现截图工具
- [ ] 支持指定窗口截图
- [ ] 坐标映射工具

### Phase 4: YOLO 集成（预留）
- [ ] YOLO 模型加载
- [ ] 检测结果解析
- [ ] 坐标转换

### Phase 5: 系统集成
- [ ] 与 Test_Module 集成
- [ ] 实时检测循环
- [ ] 性能优化

## 五、交互方式设计

### 5.1 鼠标点击模式

```
增强仿真器操作：
- 按 'M' - 进入鼠标标记模式
- 点击地图 - 在点击位置生成敌人
- 按 'ESC' - 退出标记模式
```

### 5.2 键盘输入模式

```
终端输入：
请输入敌人坐标 (格式: x,y 或 x y):
> 700, 300

生成敌人 at (700, 300)
```

### 5.3 YOLO 自动检测（未来）

```
后台自动运行：
1. 每5秒截取仿真器画面
2. YOLO 检测红色圆圈（敌人）
3. 自动更新敌人位置
4. 如果敌人移动，实时更新坐标
```

## 六、坐标映射问题

### 6.1 屏幕坐标 vs 仿真坐标

```
屏幕坐标（Pygame）:
  • 原点在窗口左上角
  • 需要考虑窗口边框和标题栏
  • 可能需要减去偏移量

仿真坐标（内部）:
  • 原点在游戏区域左上角
  • 800 x 600
```

### 6.2 映射函数

```python
def screen_to_sim(screen_x, screen_y, window_offset=(0, 0)):
    """屏幕坐标 → 仿真坐标"""
    sim_x = screen_x - window_offset[0]
    sim_y = screen_y - window_offset[1]
    return sim_x, sim_y

def sim_to_screen(sim_x, sim_y, window_offset=(0, 0)):
    """仿真坐标 → 屏幕坐标"""
    screen_x = sim_x + window_offset[0]
    screen_y = sim_y + window_offset[1]
    return screen_x, screen_y
```

## 七、YOLO 模型训练（未来）

### 7.1 数据收集

```
从仿真器截图收集训练数据：
1. 随机生成不同位置的敌人
2. 截取仿真器画面
3. 标注敌人位置（红色圆圈）
4. 生成 YOLO 格式标注文件
```

### 7.2 模型训练

```bash
# 使用 YOLOv8 训练
yolo detect train data=dataset.yaml model=yolov8n.pt epochs=100
```

### 7.3 模型优化

- 针对红色圆圈特征优化
- 考虑不同大小和颜色
- 处理遮挡情况

## 八、测试计划

### 测试1: 鼠标点击
```
1. 启动仿真器
2. 按 'M' 进入标记模式
3. 点击地图 (700, 300)
4. 验证敌人生成在正确位置
```

### 测试2: 键盘输入
```
1. 输入坐标 "700, 300"
2. 验证敌人生成
3. 验证坐标准确
```

### 测试3: YOLO 检测（未来）
```
1. 生成敌人
2. 运行 YOLO 检测
3. 验证检测准确率 > 95%
```

## 九、性能指标

| 指标 | 目标 | 当前 |
|------|------|------|
| 鼠标响应时间 | < 100ms | - |
| 键盘输入响应 | < 50ms | - |
| YOLO 检测速度 | < 100ms | - |
| 检测准确率 | > 95% | - |
| 坐标转换精度 | ±5 像素 | - |

## 十、风险评估

| 风险 | 影响 | 缓解措施 |
|------|------|----------|
| 坐标映射错误 | 高 | 详细测试和校准 |
| YOLO 检测失败 | 中 | 保留手动输入备选 |
| 性能问题 | 中 | 异步处理，降低检测频率 |
| 窗口定位失败 | 低 | 多种窗口定位方法 |

## 十一、时间估算

- Phase 1: 1小时 ✅
- Phase 2: 2小时
- Phase 3: 1小时
- Phase 4: 4小时（YOLO 部分）
- Phase 5: 2小时

**总计**: 约10小时（不含 YOLO 训练）

## 十二、成功标准

✅ 支持鼠标点击设置敌人
✅ 支持键盘输入设置敌人
✅ 坐标映射准确（±5 像素）
✅ 与现有追击模块无缝集成
✅ 为 YOLO 集成预留接口
⏳ YOLO 检测准确率 > 95%（未来）
