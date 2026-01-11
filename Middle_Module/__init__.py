"""
Middle Module - 中间通信层

功能:
- 统一的通信接口
- ROS2通信
- Dora通信
- 与下层(Sim/Real)的连接

子模块:
- ROS/: ROS2通信相关代码
  - ros2_interactive_mcp.py (MCP交互)
  - ros2_robot_controller.py (机器人控制)
  - ros2_simulator.py (仿真器接口)
  - start_ros2_mcp.sh (启动脚本)
  - ros2/ (ROS2工作空间和文件)
- Dora/: Dora通信相关代码
"""

__version__ = '1.0.0'
