# Robot_Module

## 作用

`Robot_Module` 是机器人技能注册层，负责把各类能力包装成 LLM 可调用工具。
当前主入口是 `Robot_Module/skill.py`。

## 当前导航模块

- `module/navigation.py`：四足导航技能模块

当前对外保留 4 个导航技能：

- `walk`
- `climb`
- `push_box`
- `way_select`

说明：

- `walk`：沿当前路线前进
- `climb`：攀爬箱子或高台，单步上限 `0.3 m`
- `push_box`：把可推动箱子移到目标位置
- `way_select`：从中间位置切到左侧或右侧路线

## 当前执行方式

- 通过 FastMCP 注册工具
- 技能执行结果统一封装为结构化反馈
- 默认等待执行反馈 `20 秒`
- `module/navigation.py` 默认直接写 IsaacLab EnvTest 控制文件
- 可选执行后端：
  - `file`：默认，直接写 `/tmp/*.txt`
  - `udp`：发到 `Socket/envtest_socket_server.py`
  - `ros`：继续走 `Comm_Module/execution_comm.py`
- `way_select` 默认按侧向 `walk` 控制；如需切到导航策略可设置：
  - `export FINALPROJECT_WAY_SELECT_POLICY=navigation`
- `push_box.target_position` 支持：
  - `"auto"`
  - `"x,y,z"`
  - `"[x, y, z]"`

## 主要文件

- `skill.py`：注册所有工具并暴露给上层
- `module/navigation.py`：导航技能实现
- `module/example.py`：示例模块
- `module/walkisaacsim.py`：IsaacSim 相关实验模块

## 本地运行导航模块

```bash
python3 /home/xcj/work/FinalProject/Robot_Module/module/navigation.py case2
python3 /home/xcj/work/FinalProject/Robot_Module/module/navigation.py case4
```

## 最常用启动方式

默认推荐直接配合 IsaacLab 的 `file` 后端：

1. 启动 IsaacLab player

```bash
cd /home/xcj/work/IsaacLab/IsaacLabBisShe
python NewTools/envtest_model_use_player.py --scene_id 4
```

2. 启动 FinalProject 交互入口

```bash
cd /home/xcj/work/FinalProject/Interactive_Module
python interactive.py
```

如果你想单独启动技能服务器：

```bash
cd /home/xcj/work/FinalProject/Robot_Module
python skill.py
```

如果你改成 UDP 后端，再补启动：

```bash
cd /home/xcj/work/IsaacLab/IsaacLabBisShe
python Socket/envtest_socket_server.py
```

## 与上层的关系

通常由 `Interactive_Module/interactive.py` 间接调用：

- 高层 LLM 规划技能顺序
- 低层 LLM 或参数计算层给出具体参数
- `Robot_Module` 负责真正调用 `walk/climb/push_box/way_select`
