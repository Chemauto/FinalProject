# Test_Module - 目标追击功能测试

## 功能概述

本模块实现了机器人自动追击目标的功能，是双层 LLM 机器人控制系统的扩展。

### 核心功能

1. **自动追击算法**：计算目标角度和距离，自动调用旋转和前进函数
2. **敌人管理系统**：生成、管理和移动敌人目标
3. **增强仿真器**：可视化显示追击过程
4. **MCP 工具集成**：通过自然语言调用追击功能

## 目录结构

```
Test_Module/
├── plan.md                  # 详细实现计划
├── README.md                # 本文件
│
├── chase_core.py            # 追击核心算法
│   ├── ChaseController      # 追击控制器
│   ├── calculate_angle()    # 角度计算
│   ├── calculate_distance() # 距离计算
│   └── chase_step()         # 单步追击
│
├── enemy_manager.py         # 敌人管理器
│   ├── Enemy                # 敌人类
│   ├── spawn_enemy()        # 生成敌人
│   ├── move_enemy()         # 移动敌人
│   └── get_nearest_enemy()  # 获取最近敌人
│
├── enhanced_simulator.py    # 增强仿真器
│   └── EnhancedSimulator    # 仿真器主类
│
└── test_chase.py            # 测试程序
```

## 快速开始

### 1. 启动仿真器

**终端1：**
```bash
python Test_Module/enhanced_simulator.py
```

**仿真器操作：**
- 按 `R` - 生成随机敌人
- 按 `C` - 清除所有敌人
- 按 `L` - 切换追击线显示
- 按 `1-9` - 选择敌人
- 按 `ESC` - 退出

### 2. 运行测试程序

**终端2：**
```bash
python Test_Module/test_chase.py
```

**示例命令：**
```
追击坐标(700, 300)的目标
追击最近的目标
计算追击角度，我在(100, 300)朝向东方，目标在(700, 300)
```

## 算法说明

### 角度计算

```python
# 计算目标方向角度
dx = target_x - robot_x
dy = target_y - robot_y
angle_rad = atan2(-dy, dx)  # 注意：-dy 因为屏幕y向下
target_angle = degrees(angle_rad) % 360
```

### 追击流程

```
while distance > threshold:
    1. 计算目标角度
    2. 计算角度差 (target_angle - current_angle)
    3. 如果角度差 > 5°，旋转
    4. 否则，向前移动一段距离
    5. 更新位置
    6. 重复
```

## MCP 工具

### chase_target

自动追击指定坐标的目标。

```python
chase_target(
    target_x: float,      # 目标X坐标（像素）
    target_y: float,      # 目标Y坐标（像素）
    threshold: float = 20,  # 到达阈值（像素）
    step_distance: float = 0.5  # 每步距离（米）
)
```

**示例：**
```python
chase_target(target_x=700, target_y=300)
```

### chase_nearest_enemy

追击最近的敌人。

```python
chase_nearest_enemy(
    enemy_positions: str  # JSON字符串：[{"id": "1", "x": 100, "y": 200}, ...]
)
```

**示例：**
```python
positions = '[{"id": "1", "x": 100, "y": 200}, {"id": "2", "x": 500, "y": 400}]'
chase_nearest_enemy(positions)
```

### calculate_chase_angle

计算追击所需的角度（辅助工具）。

```python
calculate_chase_angle(
    robot_x: float,
    robot_y: float,
    robot_angle: float,
    target_x: float,
    target_y: float
)
```

**返回：**
```json
{
  "target_angle": 90.0,
  "angle_diff": 45.0,
  "distance_pixels": 500.0,
  "distance_meters": 5.0,
  "direction": "左转"
}
```

## 技术细节

### 坐标系统

- **屏幕坐标**：左上角 (0,0)，x 向右，y 向下
- **机器人角度**：0°=东，90°=北，180°=西，270°=南
- **比例尺**：1米 = 100像素

### 到达判断

```python
if distance_pixels < threshold:
    return "已到达目标"
```

默认阈值：20像素（0.2米）

### 敌人移动模式

- `static` - 静止不动
- `random` - 随机游走
- `flee` - 远离机器人
- `pattern` - 圆形运动

## 扩展开发

### 添加新的追击策略

在 `chase_core.py` 中扩展 `ChaseController`：

```python
async def chase_with_avoidance(self, target_pos, obstacles):
    """避障追击"""
    # 实现避障逻辑
    pass
```

### 添加新的敌人行为

在 `enemy_manager.py` 中扩展 `Enemy.move()`：

```python
elif mode == "pursue":
    # 追逐机器人
    pass
```

### 可视化增强

在 `enhanced_simulator.py` 中添加：

```python
def draw_trajectory(self, screen, points):
    """绘制追击轨迹"""
    pygame.draw.lines(screen, color, False, points, 2)
```

## 测试用例

### 测试1: 静态目标追击

```
初始: 机器人 (100, 300), angle=0°
目标: (700, 300)

预期: 角度差≈0°，前进约6米到达
```

### 测试2: 需要旋转的追击

```
初始: 机器人 (400, 500), angle=0°（朝东）
目标: (400, 100)（北方）

预期: 左转90°，前进约4米到达
```

### 测试3: 多目标追击

```
生成多个敌人，追击最近的一个
```

## 故障排查

### 问题：追击不准确

**原因**：角度计算错误

**解决**：检查 `calculate_target_angle()` 的坐标系转换

### 问题：机器人无法到达目标

**原因**：阈值太小或步长太大

**解决**：调整 `arrival_threshold` 或 `step_distance`

### 问题：异步执行阻塞

**原因**：动作函数不是异步的

**解决**：使用 `async def` 定义工具函数

## 性能优化

- 减少循环频率
- 优化角度计算
- 使用缓存避免重复计算

## 未来计划

- [ ] 添加避障功能
- [ ] 支持多机器人协同追击
- [ ] 实现预测性追击（拦截而非追赶）
- [ ] 添加路径规划算法
- [ ] 支持动态环境
- [ ] 可视化追击轨迹

---

**作者**：Claude Code
**日期**：2025-01-22
