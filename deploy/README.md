# Deploy Helpers

这里放 FinalProject 侧的部署辅助脚本。

- `go2_skill_bridge.py`：用于 IsaacLab EnvTest Sim2Sim 的 ROS2 <-> UDP 桥。
- `go2_cmd_vel_relay.py`：用于真机/legged_ros2 速度链路的最小 relay，只把 `/go2/cmd_vel` 转发到 `/rl_cmd_vel`。

`go2_skill_bridge.py` 用于先做 IsaacLab EnvTest Sim2Sim：

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

## 真机速度 relay

如果已经确认 `legged_ros2` 的 `rl_controller` 能通过 `/rl_cmd_vel` 控制 Go2，可以先只接 FinalProject 的速度命令：

```bash
cd /home/xcj/work/FinalProject
source /opt/ros/jazzy/setup.bash
python deploy/go2_cmd_vel_relay.py --verbose
```

这条链路是：

```text
FinalProject /go2/cmd_vel -> deploy/go2_cmd_vel_relay.py -> /rl_cmd_vel -> legged_ros2 rl_controller
```

默认行为：

- 订阅 `/go2/cmd_vel`
- 发布 `/rl_cmd_vel`
- 不修改 `legged_ros2` 的 `cmd_vel_topic` 默认配置
- 不发布 `/go2/odom`、`/go2/skill_status`、`/go2/scene_objects`
- 输入订阅默认使用 `best_effort` QoS，兼容常见 `ros2 topic pub` / 真机发布端

因此这条链路只适合先验证速度动作。FinalProject 下发的速度会让机器人动，但如果没有额外状态 topic，执行验收仍可能失败。

如果输入端需要 reliable QoS，可以切换：

```bash
python deploy/go2_cmd_vel_relay.py --input-qos reliable --verbose
```

可选限幅：

```bash
python deploy/go2_cmd_vel_relay.py \
  --max-linear-speed 0.5 \
  --max-angular-speed 1.0 \
  --verbose
```

### relay 验证顺序

按三个终端同时运行，顺序不要反：

1. 启动 relay：

```bash
cd /home/xcj/work/FinalProject
source /opt/ros/jazzy/setup.bash
python deploy/go2_cmd_vel_relay.py --verbose
```

2. 监听输出：

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /rl_cmd_vel
```

3. 持续发布输入：

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub -r 10 /go2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

relay 终端应输出类似：

```text
cmd_vel -> linear=(0.500, 0.000, 0.000), angular=(0.000, 0.000, 0.000)
```

停止：

```bash
ros2 topic pub --once /go2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

如果 `/go2/cmd_vel` 有消息但 relay 没有打印，先试：

```bash
python deploy/go2_cmd_vel_relay.py --input-qos reliable --verbose
```

如果 relay 有打印但 `/rl_cmd_vel` 没显示，优先确认监听终端是在 relay 启动后再打开，且发布端使用 `-r 10` 持续发送，而不是只发一次。
