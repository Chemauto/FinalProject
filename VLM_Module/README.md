# VLM_Module

## 作用

`VLM_Module/vlm_core.py` 负责读取提示词、接收图片并调用远程视觉模型，输出结构化环境事实。
`VLM_Module/image_source.py` 负责取图：优先摄像头，没有则回退到 `VLM_Module/assets`。

## 依赖

- `openai`
- `pyyaml`
- 可选：`opencv-python`
- 项目根目录 `.env` 中需要有 `Test_API_KEY`

## 使用

在项目根目录运行：

```bash
python3 VLM_Module/vlm_core.py
```

运行时会先尝试读取摄像头；如果没有可用摄像头，就读取 `VLM_Module/assets` 中的默认图片。

也可以固定测试某张图片：

```bash
python3 -c "from VLM_Module.vlm_core import VLMCore; print(VLMCore().describe('VLM_Module/assets/3.png'))"
```

## 输入输出

- 输入：摄像头图片、默认图片，或代码中传入的图片路径
- 输出：终端打印 JSON 字符串，包含 `ground`、`left_side`、`right_side`、`front_area`、`obstacles`、`suspected_height_diff`、`uncertainties`
