# Sim to Real 部署指南

本文档描述从 IsaacLab 仿真环境切换到 Unitree Go2 真机部署时，每个 ROS2 topic 需要传输什么数据、什么格式、什么频率，以及代码中哪里消费这些数据。

## 概览

仿真和真机之间有 8 个 ROS2 topic 需要对接：

| 方向 | Topic | 消息类型 | 说明 |
|------|-------|---------|------|
| 真机→代码 | `/go2/odom` | `nav_msgs/Odometry` | 机器人位姿 |
| 真机→代码 | `/go2/skill_status` | `std_msgs/String` (JSON) | 技能执行状态回传 |
| 真机→代码 | `/go2/scene_objects` | `std_msgs/String` (JSON) | 场景物体检测结果 |
| 真机→代码 | `/go2/state` | 自定义 | 机器人整体状态（当前未使用） |
| 真机→代码 | `/go2/wrist_camera/image_raw` | `sensor_msgs/Image` | 腕部摄像头图像 |
| 代码→真机 | `/go2/skill_command` | `std_msgs/String` (JSON) | 技能命令下发 |
| 代码→真机 | `/go2/cmd_vel` | `geometry_msgs/Twist` | 速度命令 |
| 代码→真机 | `/go2/goal_pose` | `geometry_msgs/PoseStamped` | 导航目标点 |

---

## 一、真机→代码（订阅方向）

### 1. `/go2/odom` — 机器人位姿

**消息类型：** `nav_msgs/Odometry`

**代码消费位置：** `Hardware_Module/backends/go2/data.py` → `_odom_callback()`

**代码只读取 `pose.pose.position` 的三个分量：**

```
msg.pose.pose.position.x  →  robot_x
msg.pose.pose.position.y  →  robot_y
msg.pose.pose.position.z  →  robot_z
```

**内部组装为：**

```json
{
  "odom": {
    "position": [x, y, z]
  }
}
```

**对应仿真中的数据：** 仿真通过 `/proc` 找进程后读取 `/tmp/envtest_state.json` 中的 `robot_pose` 字段，格式为 `[x, y, z]`。代码在 `Hardware_Module/backends/sim/data.py` 的 `_build_live_snapshot()` 中组装。

**发布频率建议：** ≥10 Hz（执行层 0.5 秒轮询一次，但需要保证每次轮询能拿到最新数据）

---

### 2. `/go2/skill_status` — 技能执行状态回传

**消息类型：** `std_msgs/String`（payload 是 JSON 字符串）

**代码消费位置：** `Hardware_Module/backends/go2/data.py` → `_skill_callback()`

**JSON payload 完整格式：**

```json
{
  "skill": "navigation",
  "model_use": 4,
  "start": true,
  "goal": [6.0, 0.0, 0.0],
  "vel_command": [0.3, 0.0, 0.0],
  "scene_id": 3
}
```

**各字段说明：**

| 字段 | 类型 | 必须提供 | 说明 |
|------|------|---------|------|
| `skill` | string | 否 | 当前执行的技能名，如 `"navigation"`, `"walk"`, `"climb"` |
| `model_use` | int | 是 | 技能编码。0=idle, 1=walk, 2=climb, 3=push_box, 4=navigation, 5=nav_climb |
| `start` | bool | 是 | 当前技能是否正在执行。执行层用这个判断技能是否被外部停止 |
| `goal` | `[float, float, float]` | 条件必须 | 导航目标点。当 model_use 为 3/4/5 时必须有值 |
| `vel_command` | `[float, float, float]` | 否 | 当前速度命令 `[vx, vy, vz]`，仅 walk/climb 技能使用 |
| `scene_id` | int | 否 | 当前场景编号 |

**代码内部读取方式：** 直接 `json.loads(msg.data)`，然后按 key 取值。

**对应仿真中的数据：** 仿真读取 `/tmp/envtest_status.json`，代码在 `Hardware_Module/backends/sim/data.py` 的 `_build_live_snapshot()` 中组装同样的字段。

**关键约束：**
- 执行层（`Excu_Module/state.py`）通过 `runtime.start` 判断技能是否被停止。如果 `start` 从 `true` 变为 `false`，当前轮询循环会判定为 FAILURE。
- 执行层通过 `runtime.model_use` 判断技能模式是否发生变化。
- 导航技能（model_use 4, 5）用 `runtime.goal` 与 `observation.agent_position` 计算距离，判断是否到达。

**发布频率建议：** ≥10 Hz（与 odom 同步即可）

---

### 3. `/go2/scene_objects` — 场景物体检测结果

**消息类型：** `std_msgs/String`（payload 是 JSON 数组字符串）

**代码消费位置：** `Hardware_Module/backends/go2/data.py` → `_scene_callback()` + `sync_object_facts_from_live_data()`

**JSON payload 完整格式：**

```json
[
  {
    "id": "platform_1",
    "type": "platform",
    "center": [3.0, 0.75, 0.125],
    "size": [0.6, 0.8, 0.25],
    "movable": false
  },
  {
    "id": "box",
    "type": "box",
    "center": [1.7, 0.0, 0.15],
    "size": [0.3, 0.3, 0.3],
    "movable": true
  }
]
```

**各字段说明：**

| 字段 | 类型 | 必须提供 | 说明 |
|------|------|---------|------|
| `id` | string | 是 | 物体唯一标识。用于规则覆盖判断（如 `"platform_1"`, `"box"`）|
| `type` | string | 是 | 物体类型。当前识别 `"platform"` 和 `"box"` 两种 |
| `center` | `[float, float, float]` | 是 | 物体中心坐标 `[x, y, z]`。z 为物体底面高度 |
| `size` | `[float, float, float]` | 是 | 物体尺寸 `[宽x, 深y, 高z]` |
| `movable` | bool | 否 | 是否可移动。只有 `box` 为 `true`，默认 `false` |

**对应仿真中的数据：** 仿真从 `/tmp/envtest_assets.json` 读取物体信息，代码在 `_parse_asset()` 和 `_build_runtime_objects()` 中组装同样的格式。

**代码内部读取方式：** `json.loads(msg.data)` 得到列表，在 `sync_object_facts_from_live_data()`（`Hardware_Module/registry.py`）中转换为 `runtime_objects` 格式写入 `object_facts.json`。

**关键约束：**
- 规划器（`Planner_Module/planner.py`）根据物体列表判断是否触发规则覆盖（box-assisted / climbable-obstacle）。
- `box-assisted` 规则要求有且仅有一个 `type="box"` 且 `movable=true` 的物体。
- `climbable-obstacle` 规则要求有两个 `type="platform"` 的物体，且至少一个 `size[2]`（高度）≤ 0.3m。
- `push_box` 技能需要从 `runtime.envtest_alignment.box.position` 读取箱子实时位置做轮询判定。

**发布频率建议：** 1-5 Hz（物体位置不需要高频更新，但 push_box 执行期间建议 ≥2 Hz）

---

### 4. `/go2/state` — 机器人整体状态（预留）

**消息类型：** 自定义（当前代码中未订阅）

**说明：** 在 `Hardware_Module/backends/go2/schema.py` 的 `ROS2_TOPICS` 中声明了但 `data.py` 中没有创建订阅。预留用于后续扩展，比如获取机器人模式（站立/行走/趴下）、足端接触状态、IMU 数据等。

---

### 5. `/go2/wrist_camera/image_raw` — 腕部摄像头图像

**消息类型：** `sensor_msgs/Image`

**代码消费位置：** `Data_Module/image_source.py` → `_capture_from_ros2_topic()`

**要求：**
- 编码格式：`bgr8`（OpenCV BGR 格式）
- 代码使用 `cv_bridge.CvBridge` 将 ROS Image 转为 OpenCV frame，然后保存为 JPEG 文件。
- 仅当 `FINALPROJECT_ROBOT_TYPE=go2` 时才会尝试从该 topic 抓帧。
- 抓帧方式：创建临时订阅节点，`spin_once` 0.5 秒超时，取回调中最新的一帧。

**可通过环境变量覆盖 topic 名字：**

```bash
export FINALPROJECT_VLM_ROS2_IMAGE_TOPIC=/你的实际topic
```

---

## 二、代码→真机（发布方向）

### 6. `/go2/skill_command` — 技能命令下发

**消息类型：** `std_msgs/String`（payload 是 JSON 字符串）

**代码发布位置：** `Excu_Module/runtime.py` → `_ros2_publish_command()`

**JSON payload 格式（所有字段都是可选的，按需组合）：**

```json
{
  "model_use": 4,
  "start": true,
  "goal": [6.0, 0.0, 0.0],
  "velocity": [0.3, 0.0, 0.0],
  "reset": 0
}
```

**各字段说明：**

| 字段 | 类型 | 出现场景 | 说明 |
|------|------|---------|------|
| `model_use` | int | 所有技能 | 技能编码。0=idle, 1=walk, 2=climb, 3=push_box, 4=navigation, 5=nav_climb |
| `start` | bool | 所有技能 | `true`=开始执行, `false`=停止 |
| `goal` | `[float, float, float]` | push_box, navigation, nav_climb, climb_align | 目标坐标 |
| `velocity` | `[float, float, float]` | walk, climb | 速度命令 `[vx, vy, vz]` |
| `reset` | int | 重置场景 | 0=不重置, 1=重置 |

**典型命令序列示例（navigation 技能）：**

```
时间 0:   {"model_use": 4, "start": true, "goal": [6.0, 0.0, 0.0]}   ← 开始导航
时间 ...: (轮询等待到达)
时间 T:   {"start": false}                                             ← 停止
时间 T+:  {"model_use": 0, "velocity": [0, 0, 0]}                    ← 回到 idle
```

**典型命令序列示例（walk 技能）：**

```
时间 0:   {"model_use": 1, "start": true, "velocity": [0.3, 0, 0]}    ← 开始前进
时间 T:   {"start": false}                                             ← 停止
时间 T+:  {"model_use": 0, "velocity": [0, 0, 0]}                    ← 回到 idle
```

**代码会同时发布到 `/go2/cmd_vel` 或 `/go2/goal_pose`，所以该 topic 上的 `goal` 和 `velocity` 字段可以视为冗余的完整状态记录。真机端可以选择只监听这个 topic 来获取所有命令信息。**

---

### 7. `/go2/cmd_vel` — 速度命令

**消息类型：** `geometry_msgs/Twist`

**代码发布位置：** `Excu_Module/runtime.py` → `_ros2_publish_command()`

**只有当命令中包含 `velocity` 字段时才发布。**

**字段映射：**

```
velocity[0]  →  twist.linear.x   (前进速度，m/s)
velocity[1]  →  twist.linear.y   (侧移速度，m/s，当前所有技能都为 0)
velocity[2]  →  twist.linear.z   (垂直速度，m/s，当前所有技能都为 0)
```

**使用技能：** `walk`（model_use=1）、`climb`（model_use=2）

**停止命令：** `[0.0, 0.0, 0.0]`

---

### 8. `/go2/goal_pose` — 导航目标点

**消息类型：** `geometry_msgs/PoseStamped`

**代码发布位置：** `Excu_Module/runtime.py` → `_ros2_publish_command()`

**只有当命令中包含 `goal` 字段且不是 `"auto"` 时才发布。**

**字段映射：**

```
goal[0]      →  pose.pose.position.x    (目标 x)
goal[1]      →  pose.pose.position.y    (目标 y)
goal[2]      →  pose.pose.position.z    (目标 z)
goal[3](可选) →  pose.pose.orientation  (yaw → quaternion)
                orientation.z = sin(yaw/2)
                orientation.w = cos(yaw/2)
```

**`header.frame_id` 固定为 `"map"`。**

**使用技能：** `push_box`（model_use=3）、`navigation`（model_use=4）、`nav_climb`（model_use=5）、`climb_align`（model_use=4）

**典型值示例：**

```
goal = [6.0, 0.0, 0.0]        →  导航到 (6, 0, 0)
goal = [1.7, 0.0, 0.3]        →  推箱子到 (1.7, 0, 0.3)
goal = [1.05, 0.0, 0.0, 0.0]  →  爬对齐到 (1.05, 0, 0)，朝向 0
```

---

## 三、model_use 编码对照表

这是所有技能使用的编码，真机端和代码端必须一致：

| model_use | 技能名 | 命令类型 | 执行策略 |
|-----------|--------|---------|---------|
| 0 | idle | — | 空闲状态，不发运动命令 |
| 1 | walk | velocity `[vx, 0, 0]` | 超时等待 + 事后校验 |
| 2 | climb | velocity `[vx, 0, 0]` | 超时等待 + 事后校验 |
| 3 | push_box | goal `[x, y, z]` | 0.5s 轮询检测箱子到位 (阈值 0.08m) |
| 4 | navigation | goal `[x, y, z]` | 0.5s 轮询检测到达 (阈值 0.13m) |
| 5 | nav_climb | goal `[x, y, z]` | 0.5s 轮询检测到达 (阈值 0.13m) |

注意：`climb_align` 也使用 model_use=4（navigation 编码），因为它本质是导航到对齐点。

---

## 四、真机端需要实现的功能

### 4.1 必须实现

1. **odom 发布节点** — 将机器人里程计数据发布到 `/go2/odom`
   - 数据来源：Go2 自带的 `/odom` topic 或底盘 SDK
   - 只需要 position (x, y, z)

2. **skill_status 发布节点** — 将技能状态回传到 `/go2/skill_status`
   - 收到 `/go2/skill_command` 后解析 JSON，执行对应动作
   - 执行过程中持续发布当前状态（model_use, start, goal 等）
   - 技能完成后将 `start` 置为 `false`

3. **cmd_vel 订阅节点** — 接收速度命令驱动底盘运动
   - walk 和 climb 技能通过此 topic 下发速度

4. **goal_pose 订阅节点** — 接收导航目标点
   - push_box / navigation / nav_climb / climb_align 通过此 topic 下发目标

### 4.2 需要感知管线

5. **scene_objects 发布节点** — 物体检测结果发布到 `/go2/scene_objects`
   - 数据来源：LiDAR 点云聚类 / 深度相机 + YOLO/SAM 检测
   - 必须提供：`id`, `type`, `center`, `size`, `movable`
   - 这是当前最大的缺口。仿真中物体坐标由仿真器直接提供，真机需要完整的感知管线。

### 4.3 可选

6. **wrist_camera 发布节点** — 腕部摄像头图像发布到 `/go2/wrist_camera/image_raw`
   - 如果 VLM 不需要真机图像（比如用 USB 摄像头代替），可以不实现
   - 可以通过 `FINALPROJECT_VLM_IMAGE_PATH` 环境变量指定固定路径的图片来绕过

---

## 五、环境变量配置

真机部署时需要设置：

```bash
# 指定机器人类型（切换到 Go2 后端）
export FINALPROJECT_ROBOT_TYPE=go2

# 指定执行后端为 ROS2
export FINALPROJECT_NAV_BACKEND=ros

# （可选）覆盖默认 topic 名字
export FINALPROJECT_VLM_ROS2_IMAGE_TOPIC=/你的实际摄像头topic

# （可选）调整轮询频率和到达阈值
export FINALPROJECT_STATUS_POLL_SEC=0.5
```

---

## 六、代码文件对照

| 文件 | 职责 | 关联 topic |
|------|------|-----------|
| `Hardware_Module/backends/go2/schema.py` | 状态格式定义、topic 名字、`TASK_DATA` 注册 | 所有订阅 topic |
| `Hardware_Module/backends/go2/data.py` | ROS2 订阅节点、数据缓存、`get_data()` 接口 | `/go2/odom`, `/go2/skill_status`, `/go2/scene_objects` |
| `Excu_Module/runtime.py` | ROS2 命令发布、`_ros2_publish_command()` | `/go2/skill_command`, `/go2/cmd_vel`, `/go2/goal_pose` |
| `Data_Module/image_source.py` | ROS2 图像抓帧 | `/go2/wrist_camera/image_raw` |
| `Excu_Module/state.py` | 轮询判定（读取 `Hardware_Module.get_state()` 统一状态） | 间接消费所有订阅 topic |
| `Excu_Module/executor.py` | 执行编排（调用 runtime.py 下发命令） | 所有发布 topic |
| `Hardware_Module/schema.py` | 统一状态解析入口 `get_state()` | 间接消费所有订阅 topic |

---

## 七、仿真→真机映射关系

| 仿真数据来源 | 仿真文件路径 | 真机 ROS2 topic |
|-------------|------------|----------------|
| 机器人位姿 | `/tmp/envtest_state.json` → `robot_pose` | `/go2/odom` → `pose.pose.position` |
| 技能编码 | `/tmp/envtest_status.json` → `model_use` | `/go2/skill_status` → `model_use` |
| 执行状态 | `/tmp/envtest_status.json` → `start` | `/go2/skill_status` → `start` |
| 导航目标 | `/tmp/envtest_status.json` → `goal` | `/go2/skill_status` → `goal` |
| 物体列表 | `/tmp/envtest_assets.json` | `/go2/scene_objects` |
| 命令下发 | 写入 `/tmp/envtest_model_use.txt` 等 | `/go2/skill_command` + `/go2/cmd_vel` + `/go2/goal_pose` |
| 摄像头图像 | `/tmp/envtest_front_camera.png` | `/go2/wrist_camera/image_raw` |
