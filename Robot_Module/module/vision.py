"""
视觉感知模块 (Vision Perception Module)

基于VLM的颜色识别和动作执行功能。
"""

import sys
import json

# 全局动作队列
_action_queue = None

# VLM核心实例
_vlm_core = None


def _get_action_queue():
    """获取动作队列（懒加载：首次使用时自动初始化 ROS 队列）"""
    global _action_queue
    if _action_queue is None:
        from pathlib import Path
        project_root = Path(__file__).parent.parent.parent
        sys.path.insert(0, str(project_root))
        from ros_topic_comm import get_shared_queue
        _action_queue = get_shared_queue()
        print("[vision.py] ROS队列已自动初始化", file=sys.stderr)
    return _action_queue


def _get_vlm_core():
    """获取VLM核心实例（懒加载：首次使用时自动初始化）"""
    global _vlm_core
    if _vlm_core is None:
        from pathlib import Path
        project_root = Path(__file__).parent.parent.parent
        sys.path.insert(0, str(project_root))
        from VLM_Module.vlm_core import VLMCore
        _vlm_core = VLMCore()
        print("[vision.py] VLM核心已初始化", file=sys.stderr)
    return _vlm_core


# ==============================================================================
# MCP 注册函数
# ==============================================================================

def register_tools(mcp):
    """
    注册视觉感知相关的所有工具函数到 MCP 服务器

    Args:
        mcp: FastMCP 服务器实例

    Returns:
        工具函数字典 {name: function}
    """

    @mcp.tool()
    async def detect_color_and_act(image_path: str = None) -> str:
        """检测图片颜色并执行相应动作

        识别图片中的方块颜色，根据颜色执行对应动作：
        - 红色：向前移动1米
        - 橙色：向前移动1米
        - 黄色：左转90度
        - 绿色：向后移动1米
        - 蓝色：右转90度
        - 紫色：停止
        - 黑色：无动作

        重要：如果用户指令中提到了图片路径，必须将完整路径作为image_path参数传入！

        Args:
            image_path: 图片文件路径（可选）。如果用户提供了路径，必须使用该路径；否则使用默认图片。

        Returns:
            动作指令JSON字符串

        Examples:
            detect_color_and_act()  # 使用默认图片
            detect_color_and_act('/home/robot/image.png')  # 使用用户指定的图片
            detect_color_and_act('/home/robot/work/FinalProject/VLM_Module/assets/green.png')
        """
        print(f"[vision.detect_color_and_act] 图片路径: {image_path or '默认'}", file=sys.stderr)

        vlm = _get_vlm_core()
        result = vlm.perceive(image_path)

        if not result:
            print("[vision.detect_color_and_act] 未检测到有效颜色", file=sys.stderr)
            return json.dumps({"action": "none", "reason": "未检测到有效颜色"}, ensure_ascii=False)

        action = result.get('action')

        # 根据动作类型执行命令
        if action == 'move_forward':
            cmd = {'action': 'move_forward', 'parameters': {'distance': 1.0, 'speed': 0.3}}
            print("[vision.detect_color_and_act] 检测到红色/橙色，执行前进", file=sys.stderr)
        elif action == 'move_backward':
            cmd = {'action': 'move_backward', 'parameters': {'distance': 1.0, 'speed': 0.3}}
            print("[vision.detect_color_and_act] 检测到绿色，执行后退", file=sys.stderr)
        elif action == 'turn_left':
            cmd = {'action': 'turn', 'parameters': {'angle': 90.0, 'angular_speed': 0.5}}
            print("[vision.detect_color_and_act] 检测到黄色，执行左转", file=sys.stderr)
        elif action == 'turn_right':
            cmd = {'action': 'turn', 'parameters': {'angle': -90.0, 'angular_speed': 0.5}}
            print("[vision.detect_color_and_act] 检测到蓝色，执行右转", file=sys.stderr)
        elif action == 'stop':
            cmd = {'action': 'stop', 'parameters': {}}
            print("[vision.detect_color_and_act] 检测到紫色，执行停止", file=sys.stderr)
        else:
            print(f"[vision.detect_color_and_act] 未知动作: {action}", file=sys.stderr)
            return json.dumps({"action": "none", "reason": f"未知动作: {action}"}, ensure_ascii=False)

        # 发送到仿真器
        _get_action_queue().put(cmd)

        return json.dumps(cmd, ensure_ascii=False)

    print("[vision.py:register_tools] 视觉感知模块已注册 (1 个工具)", file=sys.stderr)

    # 返回工具函数字典
    return {
        'detect_color_and_act': detect_color_and_act
    }
