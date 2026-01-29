"""
IsaacSim 行走控制模块 (Walk IsaacSim Module)

通过 Socket UDP 控制 IsaacSim 中的机器人执行基于 RL 策略的行走动作。
实现方式参考 /home/xcj/work/IsaacLab/IsaacLabBisShe/Socket/send_cmd.py
采用持续发送命令机制，将任务分解为多个小步骤执行。

使用方法:
    1. 启动 IsaacSim:
       cd /home/xcj/work/IsaacLab/IsaacLabBisShe
       python scripts/rsl_rl/play.py --task Template-Velocity-Go2-Walk-Flat-Ros-v0 \
           --checkpoint /home/xcj/work/IsaacLab/IsaacLabBisShe/ModelBackup/WalkPolicy/WalkFlatNew.pt
    
    2. 调用 move_isaac() 函数控制机器人移动

提供 MCP 工具函数：
    - move_isaac: 统一的移动控制函数（支持前后左右移动和旋转）
    - get_isaac_config: 获取配置信息
"""

import sys
import json
import asyncio
import socket
import math
from pathlib import Path
from typing import Dict, Optional, Tuple, List
from dataclasses import dataclass

# ==============================================================================
# 配置参数
# ==============================================================================

# Socket 配置
DEFAULT_HOST = '127.0.0.1'
DEFAULT_PORT = 5555

# 默认速度配置（与 send_cmd.py 保持一致）
# 注意：速度值必须与 send_cmd.py 中的定义一致
DEFAULT_SPEEDS = {
    'forward': 0.5,      # 前进速度 (W键)
    'backward': -0.3,    # 后退速度 (S键) - 负值表示后退
    'left': 0.5,         # 左移速度 (A键)
    'right': -0.5,       # 右移速度 (D键) - 负值表示右移
    'rotate_left': 0.5,  # 左转角速度 (Q键)
    'rotate_right': -0.5, # 右转角速度 (E键) - 负值表示右转
}

# 任务分解配置
STEP_INTERVAL = 0.1    # 每步发送命令的间隔（秒），与 send_cmd.py 类似
MIN_STEP_DURATION = 0.1  # 最小步长时间

# 参数限制
MAX_DISTANCE = 10.0      # 最大移动距离（米）
MAX_ANGLE = 360.0        # 最大旋转角度（度）
MIN_SPEED = 0.1          # 最小速度
MAX_SPEED = 2.0          # 最大速度
MAX_ANGULAR_SPEED = 3.14 # 最大角速度 (rad/s)

# 有效方向
VALID_DIRECTIONS = ['forward', 'backward', 'left', 'right', 'rotate_left', 'rotate_right']


# ==============================================================================
# Socket 控制器类（单例模式）
# ==============================================================================

@dataclass
class VelocityCommand:
    """速度命令数据结构"""
    vx: float = 0.0  # 前进速度 (m/s)
    vy: float = 0.0  # 横向速度 (m/s)
    wz: float = 0.0  # 旋转角速度 (rad/s)
    
    def to_socket_format(self) -> str:
        """转换为 Socket 发送格式: "vx,vy,wz"""
        return f"{self.vx:.3f},{self.vy:.3f},{self.wz:.3f}"
    
    def to_list(self) -> List[float]:
        """转换为列表格式: [vx, vy, wz]"""
        return [self.vx, self.vy, self.wz]


class IsaacSimSocketController:
    """
    IsaacSim Socket 控制器（单例模式）
    
    通过 UDP Socket 向 IsaacSim 发送速度命令，控制机器人移动。
    使用单例模式确保只有一个 Socket 连接实例。
    实现方式参考 send_cmd.py 的持续发送机制。
    """
    
    _instance = None
    _initialized = False
    
    def __new__(cls, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT):
        if IsaacSimSocketController._initialized:
            return
            
        self.host = host
        self.port = port
        self._sock = None
        self._connected = False
        
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # 不设置超时，与 send_cmd.py 一致
            self._connected = True
            print(f"[IsaacSimSocketController] Socket 已初始化 ({host}:{port})", file=sys.stderr)
        except Exception as e:
            print(f"[IsaacSimSocketController] ✗ Socket 初始化失败: {e}", file=sys.stderr)
            raise
            
        IsaacSimSocketController._initialized = True
    
    def send_command(self, vx: float, vy: float, wz: float) -> bool:
        """
        发送速度命令到 IsaacSim
        
        Args:
            vx: 前进速度 (m/s)，正值向前，负值向后
            vy: 横向速度 (m/s)，正值向左，负值向右
            wz: 旋转角速度 (rad/s)，正值左转，负值右转
            
        Returns:
            bool: 发送是否成功
        """
        if not self._connected or self._sock is None:
            print("[IsaacSimSocketController] ✗ Socket 未连接", file=sys.stderr)
            return False
        
        try:
            message = f"{vx:.3f},{vy:.3f},{wz:.3f}"
            self._sock.sendto(message.encode('utf-8'), (self.host, self.port))
            return True
        except Exception as e:
            print(f"[IsaacSimSocketController] ✗ 发送命令失败: {e}", file=sys.stderr)
            return False
    
    def send_command_list(self, cmd_list: List[float]) -> bool:
        """
        使用列表格式发送命令 [vx, vy, wz]
        与 send_cmd.py 的 current_cmd 格式一致
        """
        if len(cmd_list) >= 3:
            return self.send_command(cmd_list[0], cmd_list[1], cmd_list[2])
        return False
    
    def stop(self) -> bool:
        """发送停止命令 [0.0, 0.0, 0.0]"""
        return self.send_command(0.0, 0.0, 0.0)
    
    def close(self):
        """关闭 Socket 连接"""
        if self._sock:
            self._sock.close()
            self._sock = None
            self._connected = False
            print("[IsaacSimSocketController] Socket 已关闭", file=sys.stderr)


# 全局控制器实例（懒加载）
_controller = None

def _get_controller() -> IsaacSimSocketController:
    """获取 Socket 控制器实例（懒加载）"""
    global _controller
    if _controller is None:
        _controller = IsaacSimSocketController()
    return _controller


# ==============================================================================
# 参数验证和计算
# ==============================================================================

def _validate_parameters(direction: str, distance: float, speed: float) -> Tuple[bool, str]:
    """
    验证移动参数是否有效
    
    Returns:
        (是否有效, 错误信息)
    """
    # 验证方向
    if direction not in VALID_DIRECTIONS:
        return False, f"无效的方向 '{direction}'，可选值: {', '.join(VALID_DIRECTIONS)}"
    
    # 验证距离
    if distance <= 0:
        return False, f"距离必须大于 0，当前值: {distance}"
    
    if direction.startswith('rotate'):
        # 旋转模式：验证角度
        if distance > MAX_ANGLE:
            return False, f"旋转角度超出范围: {distance}° (最大 {MAX_ANGLE}°)"
    else:
        # 移动模式：验证距离
        if distance > MAX_DISTANCE:
            return False, f"移动距离超出范围: {distance}m (最大 {MAX_DISTANCE}m)"
    
    # 验证速度（使用绝对值）
    speed_abs = abs(speed)
    if speed_abs < MIN_SPEED:
        return False, f"速度过小: {speed} (最小 {MIN_SPEED})"
    
    if direction.startswith('rotate'):
        if speed_abs > MAX_ANGULAR_SPEED:
            return False, f"角速度超出范围: {speed} rad/s (最大 {MAX_ANGULAR_SPEED} rad/s)"
    else:
        if speed_abs > MAX_SPEED:
            return False, f"线速度超出范围: {speed} m/s (最大 {MAX_SPEED} m/s)"
    
    return True, ""


def _calculate_movement_time(direction: str, distance: float, speed: float) -> float:
    """
    计算移动所需时间
    
    Args:
        direction: 移动方向
        distance: 距离（米）或角度（度）
        speed: 速度（m/s 或 rad/s），使用绝对值计算
        
    Returns:
        float: 所需时间（秒）
    """
    speed_abs = abs(speed)
    
    if direction.startswith('rotate'):
        # 旋转：角度(度) -> 弧度 -> 时间
        angle_rad = math.radians(distance)
        return angle_rad / speed_abs
    else:
        # 线移动：距离 / 速度
        return distance / speed_abs


def _get_velocity_vector(direction: str, speed: float) -> List[float]:
    """
    根据方向获取速度向量 [vx, vy, wz]
    返回格式与 send_cmd.py 的 current_cmd 一致
    
    Returns:
        List[float]: [vx, vy, wz]
    """
    # 直接使用 DEFAULT_SPEEDS 中的值，保持与 send_cmd.py 一致
    default_speed = DEFAULT_SPEEDS.get(direction, 0.0)
    
    # 如果用户提供了 speed 参数，按比例调整
    if speed != default_speed and speed != abs(default_speed):
        # 计算比例因子
        ratio = abs(speed) / abs(default_speed) if default_speed != 0 else 1.0
        return [
            default_speed * ratio if default_speed != 0 else 0.0,
            0.0,
            0.0
        ]
    
    # 使用默认速度值
    if direction == 'forward':
        return [default_speed, 0.0, 0.0]
    elif direction == 'backward':
        return [default_speed, 0.0, 0.0]  # default_speed 已经是负值
    elif direction == 'left':
        return [0.0, default_speed, 0.0]
    elif direction == 'right':
        return [0.0, default_speed, 0.0]  # default_speed 已经是负值
    elif direction == 'rotate_left':
        return [0.0, 0.0, default_speed]
    elif direction == 'rotate_right':
        return [0.0, 0.0, default_speed]  # default_speed 已经是负值
    
    return [0.0, 0.0, 0.0]


def _decompose_task(direction: str, distance: float, speed: float) -> Tuple[List[float], int, float]:
    """
    将任务分解为多个步骤
    
    Args:
        direction: 移动方向
        distance: 距离或角度
        speed: 速度
        
    Returns:
        Tuple[List[float], int, float]: (速度向量, 步骤数, 总时间)
    """
    # 获取速度向量
    velocity_vector = _get_velocity_vector(direction, speed)
    
    # 计算总时间
    total_time = _calculate_movement_time(direction, distance, speed)
    
    # 计算步骤数（每步 0.1 秒）
    num_steps = max(1, int(total_time / STEP_INTERVAL))
    
    # 调整最后一步的时间，使总时间准确
    actual_step_time = total_time / num_steps if num_steps > 0 else total_time
    
    return velocity_vector, num_steps, actual_step_time


# ==============================================================================
# MCP 工具注册
# ==============================================================================

def register_tools(mcp):
    """注册 IsaacSim 行走控制工具到 MCP 服务器"""

    @mcp.tool()
    async def move_isaac(
        direction: str,
        distance: float = 1.0,
        speed: Optional[float] = None
    ) -> str:
        """
        控制 IsaacSim 中的机器人执行移动或旋转动作
        
        通过 Socket UDP 向 IsaacSim 持续发送速度命令，控制四足机器人执行指定的移动或旋转动作。
        实现方式参考 send_cmd.py：将任务分解为多个小步骤，每 0.1 秒发送一次命令，最后发送停止命令。
        
        前置条件：
            - 需要先启动 IsaacSim 环境
            - 启动命令：
              cd /home/xcj/work/IsaacLab/IsaacLabBisShe
              python scripts/rsl_rl/play.py --task Template-Velocity-Go2-Walk-Flat-Ros-v0 \
                  --checkpoint /home/xcj/work/IsaacLab/IsaacLabBisShe/ModelBackup/WalkPolicy/WalkFlatNew.pt
        
        Args:
            direction: 移动方向，可选值：
                      - "forward": 前进
                      - "backward": 后退
                      - "left": 左移
                      - "right": 右移
                      - "rotate_left": 左转（逆时针）
                      - "rotate_right": 右转（顺时针）
            distance: 移动距离（米）或旋转角度（度），必须 > 0，默认 1.0
            speed: 移动速度（m/s）或旋转角速度（rad/s）。
                   如果为 None，使用各方向的默认值：
                   - 前进: 0.5 m/s
                   - 后退: -0.3 m/s
                   - 左移: 0.5 m/s
                   - 右移: -0.5 m/s
                   - 左转: 0.5 rad/s
                   - 右转: -0.5 rad/s
        
        Returns:
            JSON 字符串，包含执行结果和详细信息
            
        Examples:
            # 前进 1 米，使用默认速度 0.5 m/s
            move_isaac(direction="forward", distance=1.0)
            
            # 后退 0.5 米，速度 0.3 m/s (使用绝对值)
            move_isaac(direction="backward", distance=0.5, speed=0.3)
            
            # 左移 1 米，速度 0.5 m/s
            move_isaac(direction="left", distance=1.0, speed=0.5)
            
            # 右转 90 度，角速度 0.5 rad/s
            move_isaac(direction="rotate_right", distance=90.0, speed=0.5)
            
            # 左转 45 度，使用默认角速度
            move_isaac(direction="rotate_left", distance=45.0)
        """
        print(f"[move_isaac] 开始执行 - 方向: {direction}, 距离/角度: {distance}, 速度: {speed}", 
              file=sys.stderr)
        
        # 使用默认速度
        if speed is None:
            speed = DEFAULT_SPEEDS.get(direction, 0.5)
            print(f"[move_isaac] 使用默认速度: {speed}", file=sys.stderr)
        
        # 验证参数
        is_valid, error_msg = _validate_parameters(direction, distance, speed)
        if not is_valid:
            print(f"[move_isaac] ✗ 参数验证失败: {error_msg}", file=sys.stderr)
            return json.dumps({
                "success": False,
                "error": error_msg,
                "parameters": {
                    "direction": direction,
                    "distance": distance,
                    "speed": speed
                }
            }, ensure_ascii=False)
        
        try:
            # 获取控制器
            controller = _get_controller()
            
            # 任务分解：计算速度向量、步骤数、每步时间
            velocity_vector, num_steps, step_time = _decompose_task(direction, distance, speed)
            total_time = step_time * num_steps
            
            print(f"[move_isaac] 任务分解: {num_steps} 步, 每步 {step_time:.3f}s, 总时间 {total_time:.2f}s", 
                  file=sys.stderr)
            print(f"[move_isaac] 速度向量: {velocity_vector}", file=sys.stderr)
            
            # 持续发送命令（类似 send_cmd.py 的机制）
            print(f"[move_isaac] 开始持续发送命令...", file=sys.stderr)
            
            for step in range(num_steps):
                # 发送当前命令
                controller.send_command_list(velocity_vector)
                
                # 每10步打印一次进度
                if (step + 1) % 10 == 0 or step == 0:
                    progress = (step + 1) / num_steps * 100
                    print(f"[move_isaac] 进度: {progress:.0f}% ({step+1}/{num_steps})", file=sys.stderr)
                
                # 等待步长时间
                await asyncio.sleep(step_time)
            
            # 发送停止命令
            controller.stop()
            print(f"[move_isaac] 移动完成，已发送停止命令", file=sys.stderr)
            
            # 构建返回结果
            result = {
                "success": True,
                "message": f"移动完成: {direction} {distance}{'°' if direction.startswith('rotate') else 'm'}",
                "command": {
                    "direction": direction,
                    "distance": distance,
                    "speed": speed,
                    "velocity_vector": velocity_vector
                },
                "execution": {
                    "total_steps": num_steps,
                    "step_time_seconds": round(step_time, 3),
                    "total_duration_seconds": round(total_time, 2),
                    "socket": f"{DEFAULT_HOST}:{DEFAULT_PORT}"
                }
            }
            
            return json.dumps(result, ensure_ascii=False)
            
        except Exception as e:
            print(f"[move_isaac] ✗ 执行失败: {e}", file=sys.stderr)
            # 确保发送停止命令
            try:
                controller = _get_controller()
                controller.stop()
            except:
                pass
            
            return json.dumps({
                "success": False,
                "error": str(e),
                "type": type(e).__name__,
                "parameters": {
                    "direction": direction,
                    "distance": distance,
                    "speed": speed
                }
            }, ensure_ascii=False)


    @mcp.tool()
    async def get_isaac_config() -> str:
        """
        获取 IsaacSim 行走控制的配置信息
        
        Returns:
            JSON 字符串，包含参数范围、默认值、使用说明等
            
        Examples:
            get_isaac_config()
        """
        return json.dumps({
            "success": True,
            "config": {
                "socket": {
                    "host": DEFAULT_HOST,
                    "port": DEFAULT_PORT,
                    "protocol": "UDP"
                },
                "directions": {
                    "forward": {"description": "前进", "default_speed": DEFAULT_SPEEDS['forward'], "velocity": "[0.5, 0.0, 0.0]"},
                    "backward": {"description": "后退", "default_speed": DEFAULT_SPEEDS['backward'], "velocity": "[-0.3, 0.0, 0.0]"},
                    "left": {"description": "左移", "default_speed": DEFAULT_SPEEDS['left'], "velocity": "[0.0, 0.5, 0.0]"},
                    "right": {"description": "右移", "default_speed": DEFAULT_SPEEDS['right'], "velocity": "[0.0, -0.5, 0.0]"},
                    "rotate_left": {"description": "左转", "default_speed": DEFAULT_SPEEDS['rotate_left'], "velocity": "[0.0, 0.0, 0.5]"},
                    "rotate_right": {"description": "右转", "default_speed": DEFAULT_SPEEDS['rotate_right'], "velocity": "[0.0, 0.0, -0.5]"}
                },
                "default_speeds": DEFAULT_SPEEDS,
                "parameter_ranges": {
                    "distance_linear": f"0 ~ {MAX_DISTANCE} 米",
                    "angle_rotation": f"0 ~ {MAX_ANGLE} 度",
                    "speed_linear": f"{MIN_SPEED} ~ {MAX_SPEED} m/s",
                    "speed_angular": f"{MIN_SPEED} ~ {MAX_ANGULAR_SPEED} rad/s"
                },
                "execution_config": {
                    "step_interval": STEP_INTERVAL,
                    "description": "任务分解为多个小步骤，每步持续发送命令"
                },
                "prerequisites": {
                    "isaaclab_path": "/home/xcj/work/IsaacLab/IsaacLabBisShe",
                    "model_path": "/home/xcj/work/IsaacLab/IsaacLabBisShe/ModelBackup/WalkPolicy/WalkFlatNew.pt",
                    "launch_command": "python scripts/rsl_rl/play.py --task Template-Velocity-Go2-Walk-Flat-Ros-v0 --checkpoint /home/xcj/work/IsaacLab/IsaacLabBisShe/ModelBackup/WalkPolicy/WalkFlatNew.pt"
                },
                "usage_examples": [
                    {"description": "前进 1 米", "call": 'move_isaac("forward", 1.0)'},
                    {"description": "后退 0.5 米", "call": 'move_isaac("backward", 0.5)'},
                    {"description": "左转 90 度", "call": 'move_isaac("rotate_left", 90.0)'},
                    {"description": "右移 1 米", "call": 'move_isaac("right", 1.0)'}
                ]
            }
        }, ensure_ascii=False, indent=2)


    print("[walkisaacsim.py] 已注册 2 个工具:", file=sys.stderr)
    print("  - move_isaac: 控制机器人移动/旋转（持续发送命令模式）", file=sys.stderr)
    print("  - get_isaac_config: 获取配置信息", file=sys.stderr)

    return {
        'move_isaac': move_isaac,
        'get_isaac_config': get_isaac_config
    }
