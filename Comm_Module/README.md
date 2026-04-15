# Comm_Module

## 作用

`Comm_Module` 负责项目中的通信与状态同步相关能力。

当前主要分成两部分：

- [execution_comm.py](/home/robot/work/FinalProject/Comm_Module/execution_comm.py)
  执行反馈通信
- [Status](/home/robot/work/FinalProject/Comm_Module/Status)
  EnvTest 状态同步与 `object_facts.json` 更新

## 目录结构

- [Status/envtest_status_sync.py](/home/robot/work/FinalProject/Comm_Module/Status/envtest_status_sync.py)
  状态同步核心逻辑
- [Status/sync_envtest_status.py](/home/robot/work/FinalProject/Comm_Module/Status/sync_envtest_status.py)
  手动同步脚本入口

## 当前职责

- 从 live EnvTest 进程和控制文件读取当前运行态
- 把状态写回 `config/object_facts.json`
- 支持从用户输入中提取并覆盖 `navigation_goal / pose_command / vel_command`

## 使用

手动从 live EnvTest 同步：

```bash
cd /home/xcj/work/FinalProject/Comm_Module/Status
python sync_envtest_status.py --live-envtest
```

从状态文本同步：

```bash
python sync_envtest_status.py --status-file /path/to/status.txt
```
