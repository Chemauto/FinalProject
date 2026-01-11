#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Gazebo + ROS2 四足机器人仿真启动文件

用于启动 Gazebo 仿真环境和 Unitree Go2 机器人
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    生成启动描述
    启动 Gazebo 仿真环境并加载 Unitree Go2 机器人
    """

    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='Gazebo world name'
    )

    # 优先使用我们创建的 go2_gazebo_description 包
    try:
        pkg_go2 = get_package_share_directory('go2_gazebo_description')
        print(f"✓ 找到 go2_gazebo_description 包: {pkg_go2}")
        use_custom_go2 = True
    except:
        print("⚠️  未找到 go2_gazebo_description 包，尝试使用 unitree_gazebo...")
        use_custom_go2 = False

    # 如果有自定义 go2 包，使用它
    if use_custom_go2:
        # 使用 go2_gazebo_description 的启动文件
        go2_launch = os.path.join(pkg_go2, 'launch', 'go2_gazebo.launch.py')

        if os.path.exists(go2_launch):
            print(f"✓ 使用 Go2 自定义启动文件: {go2_launch}")

            gazebo_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(go2_launch),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                }.items()
            )

            return LaunchDescription([
                world_arg,
                gazebo_launch
            ])
        else:
            print(f"⚠️  启动文件不存在: {go2_launch}")
            use_custom_go2 = False

    # 尝试使用 unitree_gazebo 包
    try:
        pkg_unitree_gazebo = get_package_share_directory('unitree_gazebo')
        print(f"✓ 找到 unitree_gazebo 包: {pkg_unitree_gazebo}")
        use_unitree_gazebo = True
    except:
        print("⚠️  未找到 unitree_gazebo 包，使用通用 Gazebo 配置")
        use_unitree_gazebo = False

    # 方式 1: 如果有 unitree_gazebo 包，使用官方启动文件
    if use_unitree_gazebo:
        # 检查是否存在 Go1/Go2 的启动文件
        go1_launch = os.path.join(pkg_unitree_gazebo, 'launch', 'go1_world.launch.py')
        go2_launch = os.path.join(pkg_unitree_gazebo, 'launch', 'go2_world.launch.py')

        if os.path.exists(go2_launch):
            launch_file = go2_launch
            robot_name = "Go2"
            print(f"✓ 使用 {robot_name} 启动文件")
        elif os.path.exists(go1_launch):
            launch_file = go1_launch
            robot_name = "Go1"
            print(f"✓ 使用 {robot_name} 启动文件")
        else:
            use_unitree_gazebo = False
            print(f"⚠️  未找到 Go1/Go2 启动文件")

    if use_unitree_gazebo:
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
        )

        return LaunchDescription([
            world_arg,
            gazebo_launch
        ])

    # 方式 2: 使用通用 Gazebo + ROS2 启动
    print("使用通用 Gazebo 配置...")

    try:
        # 尝试使用 ros_gz_sim
        gazebo_server = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('ros_gz_sim'),
                '/launch/gz_sim.launch.py'
            ]),
            launch_arguments={'gz_args': ['-r -v 4', ' empty.sdf']}.items()
        )
    except:
        # 回退到 gazebo_ros
        gazebo_server = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('gazebo_ros'),
                '/launch/gazebo.launch.py'
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items()
        )

    # 启动 spawn_entity 节点（需要 URDF/Xacro 文件）
    # 这里暂时留空，后续可以添加自定义机器人模型

    return LaunchDescription([
        world_arg,
        gazebo_server,
        # TODO: 添加机器人 spawn 节点
    ])


if __name__ == '__main__':
    generate_launch_description()
