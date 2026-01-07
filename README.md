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
    pip install openai python-dotenv pyyaml
    ```
    *   如果有其他缺失模块，直接`pip install`安装即可。
3.  **配置API Key**:
    在项目根目录下创建一个名为`.env`的文件，并在其中添加如下内容：
    ```
    Test_API_KEY="sk-your_api_key_here"
    ```
    请将`sk-your_api_key_here`替换为您从阿里云DashScope获取的真实API Key。
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

---

# ChaseBot 仿真项目

## 1. 项目概述

本项目是一个基于Dora框架的追逐机器人仿真项目。项目中包含两个主要角色：一个是作为“目标”的机器人（Target Bot），另一个是作为“追逐者”的机器人（Chase Bot）。Chase Bot 利用大型语言模型（LLM）来理解其周围环境（包括Target Bot的位置），并自主生成追逐策略。

## 2. 核心功能

-   **多机器人仿真**: 使用Dora同时管理和运行Target Bot和Chase Bot两个节点。
-   **LLM驱动的决策**: Chase Bot通过LLM分析环境状态，并决定下一步的行动（例如，前进、左转、右转）。
-   **动态追逐**: Target Bot按照预设路径移动，Chase Bot则根据实时情况动态调整自己的行为以追逐目标。
-   **模块化**: 项目结构清晰，将模拟器、LLM Agent等功能分离，通过Dora进行通信。

## 3. 运行ChaseBot仿真

1.  **环境准备**:
    请确保已完成本文档开头的“环境设置”和“环境设置 (Dora仿真)”中的所有步骤。

2.  **运行仿真**:
    所有节点都由Dora统一管理，只需一个命令即可启动。

    a. **进入 `ChaseBot_Project` 目录**:
       ```powershell
       cd D:\MyFiles\Documents\毕设相关材料\Project\ChaseBot_Project
       ```

    b. **启动Dora后台服务** (如果尚未运行):
       ```powershell
       dora up
       ```

    c. **启动ChaseBot数据流**:
       ```powershell
       dora start chase_bot.yaml --attach
       ```
       执行此命令后，仿真将在后台开始运行。您可以在终端中看到来自 `simulator` 和 `llm_agent` 节点的日志输出，展示了Chase Bot的决策过程和机器人的位置信息。