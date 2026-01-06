# Dora 交互式仿真运行指南 (UI版本)

本文档提供了运行带有图形化输入界面的Dora交互式仿真项目的最终步骤。

## 1. 环境设置

1.  **创建并激活Conda环境**:
    ```bash
    conda create -n qwen3 python=3.11 -y
    conda activate qwen3
    ```
2.  **安装依赖**:
    ```bash
    pip install openai python-dotenv pyyaml dora-rs pyarrow pygame
    ```

## 2. 运行交互式仿真

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
    -   请在 **"Robot Command Input"** 窗口的文本框中输入指令 (例如 "go forward 50cm")，然后按 `Enter` 键或点击 "Send Command" 按钮。
    -   关闭任一窗口都将结束整个程序。
