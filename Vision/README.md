# Vision

## 作用

`Vision` 负责把相机图片转成结构化环境事实。

它只做观察，不执行机器人动作，也不写 Executor 状态。

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
}
```

`scene_facts` 会包含：

- `summary`
- `terrain_features`
- `interactive_objects`
- `route_options`
- `constraints`
- `uncertainties`

## 图片来源

优先级：

```text
显式 image_path
VISION_IMAGE_PATH
FINALPROJECT_VLM_IMAGE_PATH
Vision/assets/2.png
旧项目 Data_Module/assets/2.png
本机摄像头
```
