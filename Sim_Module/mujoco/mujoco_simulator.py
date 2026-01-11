#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MuJoCo 仿真器 - Go2 四足机器人

使用 MuJoCo 物理引擎进行 3D 仿真
加载官方 Go2 URDF 模型
支持通过 ROS2 的 /cmd_vel 话题控制机器人
"""

import sys
import time
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist

# Reconfigure stdout for Windows
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

try:
    import mujoco
    mujoco_viewer = None
    try:
        import mujoco.viewer
        mujoco_viewer = mujoco.viewer
    except (ImportError, AttributeError):
        # MuJoCo viewer 可能不可用（headless 环境）
        pass
    MUJOCO_AVAILABLE = True
except ImportError:
    MUJOCO_AVAILABLE = False
    mujoco_viewer = None
    print("❌ MuJoCo 未安装")
    print("请运行: pip install mujoco")


class Go2MuJoCoNode(Node):
    """ROS2 节点，接收 cmd_vel 命令"""

    def __init__(self):
        super().__init__('go2_mujoco_controller')

        # 订阅 cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # 速度命令
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.lock = threading.Lock()

        self.get_logger().info('Go2 MuJoCo Controller Ready')

    def cmd_vel_callback(self, msg: Twist):
        """接收速度命令"""
        with self.lock:
            self.target_linear_x = float(msg.linear.x)
            self.target_angular_z = float(msg.angular.z)


class Go2MuJoCoSimulator:
    """
    Go2 四足机器人 MuJoCo 仿真器

    功能:
    - 加载官方 Go2 URDF 模型
    - 接收 /cmd_vel 命令
    - 实现简单的步态控制
    - 可视化显示
    """

    def __init__(self, model_path=None):
        if not MUJOCO_AVAILABLE:
            raise ImportError("MuJoCo 未安装，请运行: pip install mujoco")

        # 加载 MuJoCo 模型
        if model_path is None:
            # 默认使用官方 Go2 URDF
            model_path = "/home/xcj/work/FinalProject/Robot_Module/Go2_Quadruped/go2_description/urdf/go2_description.urdf"

        self.model = self.load_model(model_path)
        self.data = mujoco.MjData(self.model)

        # 机器人状态
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.angle = 0.0  # yaw 角度

        # 运动参数
        self.max_linear_velocity = 1.0  # m/s
        self.max_angular_velocity = 2.0  # rad/s

        # 步态参数
        self.gait_phase = 0.0
        self.gait_frequency = 2.0  # Hz
        self.step_height = 0.05
        self.step_length = 0.1

        # 获取关节数量和名称
        self.joint_names = []
        for i in range(self.model.njnt):
            self.joint_names.append(mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i))

        self.num_joints = len(self.joint_names)
        print(f"✓ 检测到 {self.num_joints} 个关节")

        # 站立姿态的关节角度（初始化）
        self.stance_pose = np.zeros(self.num_joints)

        # ROS2 节点
        self.ros_node = None

    def load_model(self, model_path):
        """
        加载 MuJoCo 模型

        Args:
            model_path: URDF 或 XML 文件路径
        """
        import os
        import re

        if not os.path.exists(model_path):
            raise FileNotFoundError(f"模型文件不存在: {model_path}")

        # 读取模型文件
        with open(model_path, 'r') as f:
            model_content = f.read()

        # 预处理 URDF 以移除 MuJoCo 不兼容的元素
        # MuJoCo 不支持 URDF 的某些元素，需要移除或转换

        # 1. 移除所有 <material> 标签 (MuJoCo 不支持)
        model_content = re.sub(r'<material[^>]*>.*?</material>', '', model_content, flags=re.DOTALL)
        model_content = re.sub(r'<material[^>]*/>', '', model_content)

        # 2. 移除 <material> 引用 (如 material="...")
        model_content = re.sub(r'\s+material="[^"]*"', '', model_content)

        # 3. 移除 MuJoCo 不支持的传感器标签
        model_content = re.sub(r'<sensor[^>]*>.*?</sensor>', '', model_content, flags=re.DOTALL)

        # 4. 移除 transmission 标签 (MuJoCo 不需要)
        model_content = re.sub(r'<transmission[^>]*>.*?</transmission>', '', model_content, flags=re.DOTALL)

        # 5. 移除 gazebo 相关标签
        model_content = re.sub(r'<gazebo[^>]*>.*?</gazebo>', '', model_content, flags=re.DOTALL)

        print(f"✓ URDF 预处理完成，移除 MuJoCo 不兼容元素")

        # 创建 MuJoCo 模型
        try:
            model = mujoco.MjModel.from_xml_string(model_content)
        except Exception as e:
            print(f"✗ URDF 解析失败: {e}")
            print("尝试使用内置的简化 MuJoCo 模型...")
            model = self._create_builtin_model()

        print(f"✓ MuJoCo 模型加载成功")
        print(f"  - 模型文件: {model_path}")
        print(f"  - 总自由度: {model.nv}")
        print(f"  - 位置变量: {model.nq}")
        print(f"  - 关节数: {model.njnt}")

        return model

    def _create_builtin_model(self):
        """创建内置的简化 Go2 MuJoCo 模型"""
        mjcf_xml = """<mujoco model="Go2">
  <compiler angle="radian"/>

  <worldbody>
    <body name="trunk" pos="0 0 0.4">
      <freejoint name="root"/>
      <inertial pos="0 0 0" mass="2.5" diaginertia="0.01 0.01 0.02"/>

      <!-- 前左腿 FL -->
      <body name="FL_hip" pos="0.19 0.09 0">
        <joint name="FL_hip_joint" axis="1 0 0" range="-0.5 0.5"/>
        <inertial pos="0 0 0" mass="0.1" diaginertia="0.001 0.001 0.001"/>
        <body name="FL_thigh" pos="0 0 0">
          <joint name="FL_thigh_joint" axis="0 1 0" range="-1.5 0.5"/>
          <inertial pos="0 0 -0.05" mass="0.5" diaginertia="0.005 0.005 0.01"/>
          <body name="FL_calf" pos="0 0 -0.25">
            <joint name="FL_calf_joint" axis="0 1 0" range="-2.5 0"/>
            <inertial pos="0 0 -0.1" mass="0.1" diaginertia="0.001 0.001 0.002"/>
            <geom name="FL_foot" type="sphere" size="0.02" pos="0 0 -0.2"/>
          </body>
        </body>
      </body>

      <!-- 前右腿 FR -->
      <body name="FR_hip" pos="0.19 -0.09 0">
        <joint name="FR_hip_joint" axis="1 0 0" range="-0.5 0.5"/>
        <inertial pos="0 0 0" mass="0.1" diaginertia="0.001 0.001 0.001"/>
        <body name="FR_thigh" pos="0 0 0">
          <joint name="FR_thigh_joint" axis="0 1 0" range="-1.5 0.5"/>
          <inertial pos="0 0 -0.05" mass="0.5" diaginertia="0.005 0.005 0.01"/>
          <body name="FR_calf" pos="0 0 -0.25">
            <joint name="FR_calf_joint" axis="0 1 0" range="-2.5 0"/>
            <inertial pos="0 0 -0.1" mass="0.1" diaginertia="0.001 0.001 0.002"/>
            <geom name="FR_foot" type="sphere" size="0.02" pos="0 0 -0.2"/>
          </body>
        </body>
      </body>

      <!-- 后左腿 RL -->
      <body name="RL_hip" pos="-0.19 0.09 0">
        <joint name="RL_hip_joint" axis="1 0 0" range="-0.5 0.5"/>
        <inertial pos="0 0 0" mass="0.1" diaginertia="0.001 0.001 0.001"/>
        <body name="RL_thigh" pos="0 0 0">
          <joint name="RL_thigh_joint" axis="0 1 0" range="-1.5 0.5"/>
          <inertial pos="0 0 -0.05" mass="0.5" diaginertia="0.005 0.005 0.01"/>
          <body name="RL_calf" pos="0 0 -0.25">
            <joint name="RL_calf_joint" axis="0 1 0" range="-2.5 0"/>
            <inertial pos="0 0 -0.1" mass="0.1" diaginertia="0.001 0.001 0.002"/>
            <geom name="RL_foot" type="sphere" size="0.02" pos="0 0 -0.2"/>
          </body>
        </body>
      </body>

      <!-- 后右腿 RR -->
      <body name="RR_hip" pos="-0.19 -0.09 0">
        <joint name="RR_hip_joint" axis="1 0 0" range="-0.5 0.5"/>
        <inertial pos="0 0 0" mass="0.1" diaginertia="0.001 0.001 0.001"/>
        <body name="RR_thigh" pos="0 0 0">
          <joint name="RR_thigh_joint" axis="0 1 0" range="-1.5 0.5"/>
          <inertial pos="0 0 -0.05" mass="0.5" diaginertia="0.005 0.005 0.01"/>
          <body name="RR_calf" pos="0 0 -0.25">
            <joint name="RR_calf_joint" axis="0 1 0" range="-2.5 0"/>
            <inertial pos="0 0 -0.1" mass="0.1" diaginertia="0.001 0.001 0.002"/>
            <geom name="RR_foot" type="sphere" size="0.02" pos="0 0 -0.2"/>
          </body>
        </body>
      </body>

      <!-- 机身几何体 -->
      <geom name="trunk_geom" type="box" size="0.25 0.1 0.05" rgba="0.3 0.3 0.3 1"/>
    </body>

    <!-- 地面 -->
    <geom name="floor" type="plane" size="10 10 0.1" rgba="0.5 0.5 0.5 1"/>
  </worldbody>

  <actuator>
    <position name="FL_hip_actuator" joint="FL_hip_joint" kp="100" kv="10"/>
    <position name="FL_thigh_actuator" joint="FL_thigh_joint" kp="100" kv="10"/>
    <position name="FL_calf_actuator" joint="FL_calf_joint" kp="100" kv="10"/>
    <position name="FR_hip_actuator" joint="FR_hip_joint" kp="100" kv="10"/>
    <position name="FR_thigh_actuator" joint="FR_thigh_joint" kp="100" kv="10"/>
    <position name="FR_calf_actuator" joint="FR_calf_joint" kp="100" kv="10"/>
    <position name="RL_hip_actuator" joint="RL_hip_joint" kp="100" kv="10"/>
    <position name="RL_thigh_actuator" joint="RL_thigh_joint" kp="100" kv="10"/>
    <position name="RL_calf_actuator" joint="RL_calf_joint" kp="100" kv="10"/>
    <position name="RR_hip_actuator" joint="RR_hip_joint" kp="100" kv="10"/>
    <position name="RR_thigh_actuator" joint="RR_thigh_joint" kp="100" kv="10"/>
    <position name="RR_calf_actuator" joint="RR_calf_joint" kp="100" kv="10"/>
  </actuator>
</mujoco>"""

        return mujoco.MjModel.from_xml_string(mjcf_xml)

    def set_ros_node(self, ros_node):
        """设置 ROS2 节点"""
        self.ros_node = ros_node

    def generate_trot_gait(self, dt, linear_x, angular_z):
        """
        生成 Trot 步态（对角步态）

        Args:
            dt: 时间步长
            linear_x: 前进速度
            angular_z: 转向角速度
        """
        # 更新步态相位
        self.gait_phase += self.gait_frequency * dt
        if self.gait_phase > 2 * np.pi:
            self.gait_phase -= 2 * np.pi

        # 根据 cmd_vel 调整步态
        speed_scale = min(abs(linear_x) / 0.3, 1.0)
        if abs(linear_x) < 0.01 and abs(angular_z) < 0.01:
            # 静止状态 - 保持当前姿态
            return self.data.qpos[7:].copy() if self.model.nq > 7 else self.data.qpos.copy()

        # Trot 步态相位
        # 假设关节顺序为：FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf,
        #                    RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf
        phase_fl = self.gait_phase
        phase_fr = self.gait_phase + np.pi
        phase_rl = self.gait_phase + np.pi
        phase_rr = self.gait_phase

        # 计算关节角度偏移
        joint_positions = np.zeros(self.num_joints)

        # 简化的步态：主要移动小腿关节来模拟行走
        def leg_motion(phase):
            return np.sin(phase) * 0.3 * speed_scale

        # 假设 12 个关节，每 3 个一组对应一条腿
        # FL 和 RR 一组，FR 和 RL 一组（对角）
        for i in range(min(12, self.num_joints)):
            leg_idx = i // 3  # 0=FL, 1=FR, 2=RL, 3=RR
            joint_idx = i % 3   # 0=hip, 1=thigh, 2=calf

            if joint_idx == 2:  # calf 关节
                if leg_idx == 0:  # FL
                    joint_positions[i] = leg_motion(phase_fl)
                elif leg_idx == 1:  # FR
                    joint_positions[i] = leg_motion(phase_fr)
                elif leg_idx == 2:  # RL
                    joint_positions[i] = leg_motion(phase_rl)
                elif leg_idx == 3:  # RR
                    joint_positions[i] = leg_motion(phase_rr)

        # 转向：调整髋关节
        turn_offset = np.clip(angular_z, -1.0, 1.0) * 0.3
        # 假设 hip 关节是每组的第一个 (索引 0, 3, 6, 9)
        for i in [0, 3, 6, 9]:
            if i < self.num_joints:
                if i in [0, 6]:  # FL, RL
                    joint_positions[i] += turn_offset
                else:  # FR, RR
                    joint_positions[i] -= turn_offset

        return joint_positions

    def update(self, dt=0.02):
        """
        更新仿真一步

        Args:
            dt: 时间步长 (秒)
        """
        # 从 ROS2 节点获取速度命令
        if self.ros_node is not None:
            with self.ros_node.lock:
                linear_x = self.ros_node.target_linear_x
                angular_z = self.ros_node.target_angular_z
        else:
            linear_x = 0.0
            angular_z = 0.0

        # 生成步态
        joint_positions = self.generate_trot_gait(dt, linear_x, angular_z)

        # 设置关节位置
        if self.model.nq > 7:
            # 有 floating base (7 DOF root + joints)
            self.data.qpos[7:] = joint_positions
        else:
            # 没有 floating base，直接设置
            self.data.qpos[:] = joint_positions

        # 如果有速度命令，更新机器人位置（简化版）
        if abs(linear_x) > 0.01 or abs(angular_z) > 0.01:
            self.pos_x += linear_x * dt * np.cos(self.angle)
            self.pos_y += linear_x * dt * np.sin(self.angle)
            self.angle += angular_z * dt

        # 步进仿真
        mujoco.mj_step(self.model, self.data, nstep=10)

    def launch(self, blocking=False):
        """启动 MuJoCo 查看器"""
        print("=" * 60)
        print("MuJoCo 仿真器已启动")
        print("=" * 60)
        print("控制说明:")
        print("  - 通过 /cmd_vel 话题控制机器人")
        print("  - 例如: ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}, angular: {z: 0.0}}'")
        print("  - 按 Ctrl+C 退出")
        print("=" * 60)

        # 检查 viewer 是否可用
        if mujoco_viewer is None:
            print("❌ MuJoCo Viewer 不可用")
            print("可能原因:")
            print("  - 系统无图形界面 (headless)")
            print("  - MuJoCo 版本不支持 viewer")
            print("  - 缺少 GLFW 等图形库依赖")
            print()
            print("=" * 60)
            print("使用无界面模式运行仿真")
            print("=" * 60)
            print("提示：可以通过 /cmd_vel 话题控制机器人")
            print("例如：ros2 topic pub /cmd_vel geometry_msgs/Twist \"{'linear': {'x': 0.2}, 'angular': {'z': 0.0}}\"")
            print("=" * 60)

            # 无界面模式：直接运行仿真循环
            try:
                last_print_time = time.time()
                while True:
                    self.update(dt=0.02)
                    time.sleep(0.02)

                    # 每5秒输出一次状态
                    current_time = time.time()
                    if current_time - last_print_time >= 5.0:
                        if self.ros_node is not None:
                            with self.ros_node.lock:
                                linear_x = self.ros_node.target_linear_x
                                angular_z = self.ros_node.target_angular_z
                        else:
                            linear_x = 0.0
                            angular_z = 0.0

                        print(f"✓ 仿真运行中 | 位置: ({self.pos_x:.2f}m, {self.pos_y:.2f}m) | 角度: {np.degrees(self.angle):.1f}° | cmd_vel: ({linear_x:.2f}, {angular_z:.2f})")
                        last_print_time = current_time

            except KeyboardInterrupt:
                print("\n👋 仿真已停止")
            return

        # 使用 MuJoCo 3.x 的查看器 API
        with mujoco_viewer.launch_passive(self.model, self.data) as viewer:
            # 设置相机位置
            viewer.cam.azimuth = 90
            viewer.cam.elevation = -20
            viewer.cam.distance = 2.0
            viewer.cam.lookat[:] = [0, 0, 0.3]

            try:
                while viewer.is_running():
                    # 更新仿真
                    self.update(dt=0.02)

                    # 短暂休眠，控制帧率
                    time.sleep(0.02)

            except KeyboardInterrupt:
                print("\n👋 仿真已停止")

    def run(self):
        """运行仿真（阻塞）"""
        self.launch(blocking=True)


def main():
    """主函数"""
    if not MUJOCO_AVAILABLE:
        print("❌ MuJoCo 未安装")
        print("请运行: pip install mujoco")
        return

    # 初始化 ROS2
    rclpy.init()

    # 创建 ROS2 节点
    go2_controller = Go2MuJoCoNode()

    # 创建仿真器
    simulator = Go2MuJoCoSimulator()
    simulator.set_ros_node(go2_controller)

    # 在单独的线程中运行 ROS2 spin
    executor = MultiThreadedExecutor()
    executor.add_node(go2_controller)

    # 启动 ROS2 线程
    import threading
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    try:
        # 启动仿真
        simulator.run()
    except KeyboardInterrupt:
        print("\n👋 收到退出信号")
    finally:
        go2_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
