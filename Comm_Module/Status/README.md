# Comm_Module/Status

## 作用

`Comm_Module/Status` 专门负责 EnvTest 状态同步。

这里放的是“读取 live 状态、解析状态块、更新 `object_facts.json`”相关代码，不再放在 `Interactive_Module` 里。

## 文件

- [envtest_status_sync.py](/home/robot/work/FinalProject/Comm_Module/Status/envtest_status_sync.py)
  同步核心逻辑
- [sync_envtest_status.py](/home/robot/work/FinalProject/Comm_Module/Status/sync_envtest_status.py)
  命令行脚本入口

## 典型用法

```bash
cd /home/xcj/work/FinalProject/Comm_Module/Status
python sync_envtest_status.py --live-envtest
```
