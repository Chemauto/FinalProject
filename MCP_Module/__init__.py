"""
MCP Module - Model Context Protocol 中间连接件

功能:
- 接收LLM和VLM的输入
- 从Robot_Module寻找底层技能
- 完成skill注册
- 连接上层(LLM/VLM)和下层(Middle通信层)

核心类:
- SkillRegistry: 技能注册表,管理所有可用技能
- MCPBridge: MCP桥接层,连接各模块

使用示例:
    bridge = create_mcp_bridge(robots=['Go2_Quadruped'])
    tools = bridge.get_mcp_tools_definition()
    result = bridge.execute_skill('move_forward', distance=1.0)
"""

from .mcp_bridge import MCPBridge, SkillRegistry, create_mcp_bridge

__version__ = '1.0.0'
__all__ = ['MCPBridge', 'SkillRegistry', 'create_mcp_bridge']
