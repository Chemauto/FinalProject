# VLM_Module

## 作用

`VLM_Module/vlm_core.py` 负责读取提示词、接收图片并调用远程视觉模型，输出中文图片描述。
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

## 输入输出

- 输入：摄像头图片、默认图片，或代码中传入的图片路径
- 输出：终端打印一段约 200 字的简体中文图片描述
