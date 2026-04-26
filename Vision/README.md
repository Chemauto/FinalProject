# Vision

## 作用

`Vision` 负责把相机图片和机器人实时状态转成结构化环境事实。

它只做观察，不执行机器人动作。ROS2 状态通过 WebSocket 读取，不在 Vision 内直接订阅 ROS2。

## 文件

```text
__init__.py
image_source.py
vlm.py
vlm_utils.py
prompts/VlmPrompt.yaml
```

## 输出

`observe_environment(image_path="")` 返回：

```python
{
    "status": "success",
    "image_source": "...",
    "visual_context": {...},
    "scene_facts": {...},
    "robot_state": {...},
}
```

`visual_context` 来自 VLM 图片理解。

`robot_state` 来自 `Executor.robot_ws.get_robot_state()`，包含：

- `connected`
- `robot`
- `box_world`
- `box_relative`
- `scene_objects`
- `skill_status`

`scene_facts` 会包含：

- `summary`
- `terrain_features`
- `interactive_objects`
- `route_options`
- `corridors`
- `box_step_goal`
- `constraints`
- `uncertainties`

当 WebSocket/ROS2 状态可用时，`scene_objects` 会优先融合到 `scene_facts`：

- `platform` 进入 `terrain_features`
- `box` 进入 `interactive_objects`
- 前方中心通道净宽进入 `corridors` 和 `route_options`
- 如果存在可移动箱子，临时固定输出 `box_step_goal={"x":1.7,"y":0.0,"z":0.1}`
- VLM 原始摘要保留为 `visual_summary`

`corridors` 会根据物体 `center/size` 和机器人当前位置估计前方通道：

```python
{
    "direction": "front",
    "status": "blocked",
    "clearance_y_m": 0.0,
    "required_clearance_y_m": 0.55,
    "blocking_objects": ["left", "right"],
    "reason": "前方中心线被结构化障碍物占用，机器人尺寸无法直接通过",
}
```

这用于避免 LLM 只看机器人在 `y=0` 就误判中间可直行。

如果 WebSocket 不可用，`observe_environment()` 不失败，会退回纯 VLM 结果，并设置 `robot_state.connected=False`。

## 图片来源

优先级：

```text
显式 image_path
VISION_IMAGE_PATH
FINALPROJECT_VLM_IMAGE_PATH
/tmp/envtest_front_camera.png
旧项目兼容路径 /home/xcj/work/FinalProject/Data_Module/assets/2.png
本机摄像头
```
