# Sim_Module

## 作用

`Sim_Module` 提供 2D 仿真环境，用来显示机器人运动、敌人位置和动作执行结果。
它接收机器人动作指令，并在界面中完成可视化更新。

## 依赖

- `pygame`
- 项目内的 `ros_topic_comm.py`

## 使用

在项目根目录运行：

```bash
python3 Sim_Module/sim2d/simulator.py
```

## 输入输出

- 输入：机器人动作命令、敌人位置相关事件
- 输出：2D 仿真画面、机器人状态和敌人信息
