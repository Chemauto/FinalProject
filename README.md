# 项目文件概述

## 核心文件功能介绍

*   `DemoSimple.py`: 最简单的LLM调用OpenAI接口的示例。
*   `DemoStart.py`: 加入了Prompt的示例，最终希望配置`LLM_prompts`目录下的内容。
*   `Demo2Robot.py`: 增加了JSON输出格式的一个示例。
*   `Demo2Ros.py`: 与ROS接口融合的示例。
*   `WARNROS2.py`: 一个ROS代码文件示例，需要根据实际情况编写ROS代码。
*   `2Demo2Ros.py`: 用于自娱自乐的代码，貌似可以用于测试。

## 环境设置

1.  创建并激活 Conda 环境:
    ```bash
    conda create -n qwen3 python=3.11 -y
    conda activate qwen3
    ```
2.  安装依赖:
    ```bash
    pip install openai
    pip install dotenv
    ```
    *   如果有其他缺失模块，直接`pip install`安装即可。

## 未来展望

*   Todo list: 可以考虑利用LangChain ReAct模式来增强推理效果。

---

# Dora 仿真项目

## 1. 项目概述

本项目旨在利用大型语言模型（LLM）将自然语言指令转换为机器人可执行的动作，并结合Dora框架实现对机器人模拟器的控制。通过引入图形化用户界面（UI），极大地提升了用户交互体验。

## 2. 核心功能

-   **自然语言理解**: 利用LLM将人类指令解析为结构化机器人命令。
-   **Dora集成**: 使用Dora作为通信中间件，解耦语言处理、模拟器和UI输入等模块。
-   **图形化输入界面**: 提供一个独立的UI窗口用于输入机器人指令，无需在终端中与日志混合输入。
-   **精确角度控制**: 扩展了导航指令，支持通过 `angle` 参数进行精确的角度旋转控制。

## 3. 环境设置 (Dora仿真)

在已激活的 `qwen3` Conda 环境中，安装Dora仿真所需的额外依赖：
```bash
pip install pyyaml dora-rs pyarrow pygame
```

## 4. 运行交互式Dora仿真 (UI版本)

所有节点都由Dora统一管理和启动，只需要一个终端即可完成所有操作。

1.  **进入 `Dora_Module` 目录**:
    ```powershell
    cd D:\MyFiles\Documents\毕设相关材料\Project\Dora_Module
    ```
2.  **启动Dora后台服务** (如果尚未运行):
    ```powershell
    dora up
    ```
3.  **启动完整的交互式数据流**:
    ```powershell
    dora start dora-interactive-v2.yaml --attach
    ```
    执行此命令后，将会发生：
    -   **两个窗口会自动弹出**:
        1.  一个 **Pygame 仿真窗口**，显示机器人。
        2.  一个名为 **"Robot Command Input"** 的UI窗口，用于输入指令。
    -   您的终端会显示所有节点的日志。
    -   请在 **"Robot Command Input"** 窗口的文本框中输入指令 (例如 "go forward 50cm" 或 "Turn 90 degrees left")，然后按 `Enter` 键或点击 "Send Command" 按钮。
    -   关闭任一窗口都将结束整个程序。