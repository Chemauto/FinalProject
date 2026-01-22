# 目标追击功能实现计划

## 一、需求分析

### 背景
目前系统已实现：
- ✅ 双层 LLM 架构（规划层 + 执行层）
- ✅ 自然语言控制机器人运动
- ✅ 基础运动函数（前进、后退、旋转）
- ✅ 2D Pygame 仿真环境
- ✅ 实时坐标显示

### 新需求
实现**目标追击**功能：
1. 在地图中随机生成一个敌人目标
2. 机器人能够自动追击目标
3. 已知机器人和敌人的坐标
4. 根据坐标差自动调用运动函数

## 二、技术分析

### 2.1 坐标系统
```
屏幕坐标系:
  • 原点 (0, 0) 在左上角
  • X 轴向右增长
  • Y 轴向下增长
  • 仿真器窗口: 800x600

机器人坐标系统:
  • angle = 0°  → 朝东 (向右)
  • angle = 90° → 朝北 (向上)
  • angle = 180°→ 朝西 (向左)
  • angle = 270°→ 朝南 (向下)

比例尺:
  • 1米 = 100像素
```

### 2.2 追击算法

#### 核心数学计算

**1. 计算目标方向角度**
```python
import math

# 计算从机器人到目标的向量
dx = target_x - robot_x
dy = target_y - robot_y  # 注意：屏幕坐标y向下

# 计算目标角度（弧度 → 角度）
target_angle_rad = math.atan2(-dy, dx)  # -dy 因为屏幕y向下
target_angle = math.degrees(target_angle_rad)
target_angle = target_angle % 360  # 归一化到 [0, 360)
```

**2. 计算旋转角度差**
```python
# 当前角度 vs 目标角度
angle_diff = target_angle - robot_angle

# 标准化到 [-180, 180]
angle_diff = (angle_diff + 180) % 360 - 180

# angle_diff > 0: 左转
# angle_diff < 0: 右转
```

**3. 计算距离**
```python
distance_pixels = math.sqrt(dx**2 + dy**2)
distance_meters = distance_pixels / 100  # 转换为米
```

#### 追击流程
```
while distance > threshold:
    1. 计算目标角度
    2. 计算角度差
    3. 旋转到目标方向
    4. 向前移动一段距离
    5. 等待移动完成
    6. 更新坐标（从仿真器获取）
```

## 三、系统架构设计

### 3.1 模块划分

```
Test_Module/
├── chase_core.py          # 核心追击算法
│   ├── ChaseController    # 追击控制器类
│   ├── calculate_angle()  # 计算目标角度
│   ├── calculate_distance() # 计算距离
│   └── chase_step()       # 单步追击
│
├── enemy_manager.py       # 敌人管理
│   ├── Enemy              # 敌人类
│   ├── spawn_enemy()      # 生成敌人
│   ├── move_enemy()       # 移动敌人（可选）
│   └── get_enemy_pos()    # 获取敌人位置
│
├── enhanced_simulator.py  # 增强仿真器
│   ├── 添加敌人显示
│   ├── 添加连线显示
│   └── 实时更新机器人状态
│
└── test_chase.py          # 测试主程序
    ├── 初始化系统
    ├── 启动仿真器
    └── 执行追击测试

Robot_Module/module/
└── chase.py               # MCP 工具封装
    └── @mcp.tool()
        chase_target(enemy_id: str) -> str
```

### 3.2 数据流

```
用户输入: "追击目标"
    ↓
[LLM 上层] 任务规划
    ↓
[LLM 下层] 调用 chase_target 工具
    ↓
[Robot_Module/module/chase.py] MCP 工具
    ↓
[Test_Module/chase_core.py] 追击控制器
    ↓
[循环]
  ├─ 计算角度 → turn()
  ├─ 计算距离 → move_forward()
  ├─ 获取机器人坐标
  └─ 获取敌人坐标
    ↓
[Simulator] 执行动作
    ↓
[完成] 追击成功
```

## 四、实现步骤

### Phase 1: 核心算法实现
- [ ] 创建 `Test_Module/chase_core.py`
- [ ] 实现角度计算函数
- [ ] 实现距离计算函数
- [ ] 实现 ChaseController 类

### Phase 2: 敌人管理
- [ ] 创建 `Test_Module/enemy_manager.py`
- [ ] 实现 Enemy 类
- [ ] 实现随机生成逻辑
- [ ] 实现敌人移动逻辑（可选）

### Phase 3: 仿真器增强
- [ ] 修改 `enhanced_simulator.py`
- [ ] 添加敌人渲染（红色圆圈）
- [ ] 添加机器人-敌人连线
- [ ] 添加追击状态显示

### Phase 4: MCP 工具注册
- [ ] 创建 `Robot_Module/module/chase.py`
- [ ] 注册 `chase_target` 工具
- [ ] 在 `skill.py` 中导入

### Phase 5: LLM 提示词
- [ ] 创建追击任务提示词
- [ ] 集成到 Interactive_Module

### Phase 6: 测试与优化
- [ ] 单元测试
- [ ] 集成测试
- [ ] 性能优化

## 五、关键技术细节

### 5.1 机器人状态获取

**问题**：如何获取机器人当前坐标？

**方案**：在仿真器中添加状态共享
```python
# simulator.py
class Robot:
    def get_position(self):
        return {'x': self.x, 'y': self.y, 'angle': self.angle}

# chase_core.py
def get_robot_position():
    # 从仿真器获取
    pass
```

### 5.2 异步执行

追击是一个长时间任务，需要：
- 异步执行基础运动函数
- 等待每个动作完成
- 防止阻塞主线程

### 5.3 到达判断

```python
THRESHOLD = 20  # 像素

if distance_pixels < THRESHOLD:
    return "已到达目标"
```

### 5.4 敌人移动（可选）

增加难度：
- 敌人随机游走
- 敌人远离机器人
- 敌人按模式移动

## 六、测试用例

### 测试1: 静态目标追击
```
初始条件:
  - 机器人: (100, 300), angle=0°
  - 敌人: (700, 300)

预期行为:
  1. 计算角度差 ≈ 0°
  2. 向前移动约6米
  3. 到达敌人附近
```

### 测试2: 需要旋转的追击
```
初始条件:
  - 机器人: (400, 500), angle=0° (朝东)
  - 敌人: (400, 100)  (北方)

预期行为:
  1. 计算角度差 ≈ 90°
  2. 左转90°
  3. 向前移动约4米
  4. 到达敌人附近
```

### 测试3: LLM调用
```
用户输入: "追击红色目标"

预期流程:
  1. LLM识别为追击任务
  2. 调用 chase_target(enemy_id="red")
  3. 自动完成追击
  4. 返回成功消息
```

## 七、扩展功能

### 7.1 多目标追击
```python
chase_nearest_enemy()  # 追击最近的目标
chase_all_enemies()    # 依次追击所有目标
```

### 7.2 避障追击
```python
chase_with_avoidance()  # 追击同时避开障碍物
```

### 7.3 可视化增强
- 显示追击轨迹
- 显示角度和距离信息
- 添加进度条

## 八、风险评估

| 风险 | 影响 | 缓解措施 |
|------|------|----------|
| 坐标系混乱 | 高 | 详细注释和单元测试 |
| 角度计算错误 | 高 | 多场景测试 |
| 异步执行阻塞 | 中 | 使用async/await |
| 性能问题 | 低 | 优化循环频率 |

## 九、时间估算

- Phase 1: 2小时
- Phase 2: 1小时
- Phase 3: 2小时
- Phase 4: 1小时
- Phase 5: 1小时
- Phase 6: 2小时

**总计**: 约9小时

## 十、成功标准

✅ 机器人能够自动追击随机生成的目标
✅ 能够正确计算旋转角度
✅ 能够准确到达目标附近（20像素内）
✅ LLM能够理解"追击"指令
✅ 代码结构清晰，易于扩展
✅ 有完整的测试用例
