(qwen3) PS D:\MyFiles\Documents\毕设相关材料\Project\Dora_Module> dora start dora-interactive-mcp.yaml --attach
dataflow start triggered: 019ba0c2-5352-71bd-b75b-250b7a3b6e57
llm-agent-mcp: INFO   spawner    spawning: "D:\\MyAPP\\Miniconda\\Miniconda3py313\\envs\\qwen3\\python.exe" -u \\?\D:\MyFiles\Documents\毕设相关材料\Project\MCP_Server\llm_agent_with_mcp.py
simulator: INFO   spawner    spawning: "D:\\MyAPP\\Miniconda\\Miniconda3py313\\envs\\qwen3\\python.exe" -u \\?\D:\MyFiles\Documents\毕设相关材料\Project\Dora_Module\simulator.py
ui-input-node: INFO   spawner    spawning: "D:\\MyAPP\\Miniconda\\Miniconda3py313\\envs\\qwen3\\python.exe" -u \\?\D:\MyFiles\Documents\毕设相关材料\Project\Dora_Module\input_ui.py
INFO   dora daemon    finished building nodes, spawning...
llm-agent-mcp: INFO   spawner    spawning `D:\MyAPP\Miniconda\Miniconda3py313\envs\qwen3\python.exe` in `D:\MyFiles\Documents\毕设相关材料\Project\Dora_Module`
simulator: INFO   spawner    spawning `D:\MyAPP\Miniconda\Miniconda3py313\envs\qwen3\python.exe` in `D:\MyFiles\Documents\毕设相关材料\Project\Dora_Module`
ui-input-node: INFO   spawner    spawning `D:\MyAPP\Miniconda\Miniconda3py313\envs\qwen3\python.exe` in `D:\MyFiles\Documents\毕设相关材料\Project\Dora_Module`
ui-input-node: INFO   daemon    node is ready
simulator: stdout    D:\MyAPP\Miniconda\Miniconda3py313\envs\qwen3\Lib\site-packages\pygame\pkgdata.py:25: UserWarning: pkg_resources is deprecated as an API. See https://setuptools.pypa.io/en/latest/pkg_resources.html. The pkg_resources package is slated for removal as early as 2025-11-30. Refrain from using this package or pin to Setuptools<81.
simulator: stdout      from pkg_resources import resource_stream, resource_exists
simulator: stdout    pygame 2.6.1 (SDL 2.28.4, Python 3.11.14)
simulator: stdout    Hello from the pygame community. https://www.pygame.org/contribute.html
simulator: INFO   daemon    node is ready
llm-agent-mcp: stdout    INFO: rospy not installed. ROS1 adapter will be disabled.
llm-agent-mcp: INFO   daemon    node is ready
INFO   daemon    all nodes are ready, starting dataflow
llm-agent-mcp: stdout          To enable ROS1 support, install ROS1: http://wiki.ros.org/ROS/Installation
llm-agent-mcp: stdout    LLM_AGENT_MCP: Node initialized. Waiting for user commands...
simulator: stdout    SIM: Simulator running. Waiting for Dora commands...
ui-input-node: stdout    UI Input Node is running.
llm-agent-mcp: stdout    [DoraAdapter] Failed to initialize: Could not initiate node from environment variable. For dynamic node, please add a node id in the initialization function.
llm-agent-mcp: stdout
llm-agent-mcp: stdout    Caused by:
llm-agent-mcp: stdout       0: failed to set up tracing subscriber
llm-agent-mcp: stdout       1: failed to set tracing global subscriber for llm-agent-mcp
llm-agent-mcp: stdout       2: a global default trace dispatcher has already been set
llm-agent-mcp: stdout
llm-agent-mcp: stdout    Location:
llm-agent-mcp: stdout        libraries\extensions\telemetry\tracing\src\lib.rs:116:59
llm-agent-mcp: stdout    LLM_AGENT_MCP: Dora adapter available: False
ui-input-node: stdout    UI sending command: 往前走1米
llm-agent-mcp: stdout
llm-agent-mcp: stdout    ============================================================
llm-agent-mcp: stdout    LLM_AGENT_MCP: Received command: '往前走1米'
llm-agent-mcp: stdout    LLM_AGENT_MCP: Calling LLM with MCP tools...
llm-agent-mcp: stdout    LLM_AGENT_MCP: LLM wants to call 1 tool(s)
llm-agent-mcp: stdout    LLM_AGENT_MCP: Calling move_forward with args: {'distance': 1, 'unit': 'm'}
llm-agent-mcp: stdout    [Robot Skill] move_forward: 1m
llm-agent-mcp: stdout    LLM_AGENT_MCP: Result: {'success': True, 'action': 'move_forward', 'distance': 1, 'unit': 'm', 'result': {'success': False, 'error': 'Dora adapter not available'}}