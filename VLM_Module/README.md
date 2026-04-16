# VLM_Module

## 作用

`VLM_Module/vlm_core.py` 负责读取提示词、接收图片并调用远程视觉模型，输出结构化视觉语义。
`VLM_Module/image_source.py` 负责取图：优先读取 live EnvTest 导出的 `/tmp/envtest_front_camera.png`，其次摄像头，最后回退到 `VLM_Module/assets`。

当前视觉工具注册路径是：

```text
Robot_Module/agent_tools.py
-> 顶层 Vision tool: vlm_observe
-> Robot_Module/module/Vision/skills.py
-> Robot_Module/module/Vision/Task/Bishe/vlm_observe.py
-> 下层 Vision skill: vlm
-> VLM_Module/vlm_core.py
```

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

运行时会先尝试读取正在运行的 IsaacLab player 导出的 `/tmp/envtest_front_camera.png`；如果没有 live 图片，再尝试摄像头；如果没有可用摄像头，就读取 `VLM_Module/assets` 中的默认图片。

也可以固定测试某张图片：

```bash
python3 -c "from VLM_Module.vlm_core import VLMCore; print(VLMCore().describe('VLM_Module/assets/3.png'))"
```

## 输入输出

- 输入：摄像头图片、默认图片，或代码中传入的图片路径
- 输出：终端打印 JSON 字符串，包含 `ground`、`left_side`、`right_side`、`front_area`、`obstacles`、`suspected_height_diff`、`uncertainties`
- `VLM_Module` 不再输出 `platform_1 / platform_2 / box` 这类真值槽位；这类具体数据统一由 `Comm_Module` 提供

需要注意：

- 没开仿真也没摄像头时，VLM 会退回默认图片
- 这时得到的是默认样例图的视觉结果，不是实时环境状态
- 动作执行是否成功仍然由 `Excu_Module + Comm_Module` 判定，不由 VLM 判定

也可以通过环境变量覆盖 live 图片路径：

```bash
export FINALPROJECT_VLM_IMAGE_PATH=/tmp/envtest_front_camera.png
```
