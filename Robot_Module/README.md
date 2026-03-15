# Robot_Module

## 作用

`Robot_Module` 是机器人技能统一入口层，负责把各类能力注册为 LLM 可调用的工具。

当前入口文件：

- [skill.py](/home/xcj/work/FinalProject/Robot_Module/skill.py)

## 当前已注册模块

- [base.py](/home/xcj/work/FinalProject/Robot_Module/module/base.py)：基础底盘控制
- [chase.py](/home/xcj/work/FinalProject/Robot_Module/module/chase.py)：追击工具
- [navigation_demo.py](/home/xcj/work/FinalProject/Robot_Module/module/navigation_demo.py)：四足导航 demo

## 四足导航 Demo 工具

目前只保留 4 个与四足导航相关的工具：

- `walk`
- `climb`
- `push_box`
- `way_select`

说明：

- `walk`、`climb`、`push_box` 是基础技能
- `way_select` 是路线选择技能，内部会调用底层 `walk`
- 当前执行仍然是打印，不是真实控制
- 后续可以把这些工具替换为 IsaacLab 训练策略或 Go2 实机接口

## 本地运行 Demo

```bash
python3 Robot_Module/module/navigation_demo.py case2
python3 Robot_Module/module/navigation_demo.py case4
```
