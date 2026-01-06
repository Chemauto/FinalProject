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