# Deploy Helpers

这里放 FinalProject 侧的部署辅助脚本。当前主要脚本是 `go2_skill_bridge.py`，用于先做 IsaacLab EnvTest Sim2Sim：

```
FinalProject ROS2 Go2 后端  <->  deploy/go2_skill_bridge.py  <->  IsaacLabBisShe Socket UDP
```

## Sim2Sim 启动顺序

1. 启动 IsaacLab EnvTest player：

```bash
cd /home/xcj/work/IsaacLab/IsaacLabBisShe
python NewTools/envtest_model_use_player.py --scene_id 4
```

2. 启动 IsaacLab UDP server：

```bash
cd /home/xcj/work/IsaacLab/IsaacLabBisShe
python Socket/envtest_socket_server.py
```

3. 启动 ROS2 到 UDP 桥接器：

```bash
cd /home/xcj/work/FinalProject
source /opt/ros/humble/setup.bash
python deploy/go2_skill_bridge.py --verbose
```

4. 启动 FinalProject，强制走 Go2 ROS2 后端：

```bash
cd /home/xcj/work/FinalProject
export FINALPROJECT_ROBOT_TYPE=go2
export FINALPROJECT_NAV_BACKEND=ros
python run.py
```

## 桥接内容

- 订阅 `/go2/skill_command`，转成 EnvTest UDP 文本，例如 `model_use=4; goal=4.5,0,0.1; start=1`
- 订阅 `/go2/cmd_vel`，转成 `velocity=vx,vy,vz`
- 订阅 `/go2/goal_pose`，转成 `goal=x,y,z`
- 读取 `/tmp/envtest_live_status.json`
- 发布 `/go2/odom`
- 发布 `/go2/skill_status`
- 发布 `/go2/scene_objects`

`/go2/skill_status` 会额外带上 `envtest_alignment`，这样 FinalProject 的 `push_box` 轮询可以从 Go2 后端读到箱子实时位置。

## 常用参数

```bash
python deploy/go2_skill_bridge.py \
  --udp-host 127.0.0.1 \
  --udp-port 5566 \
  --status-json /tmp/envtest_live_status.json \
  --publish-hz 10 \
  --verbose
```

如果 ROS2 不是 Humble，替换对应的 `setup.bash` 即可。
