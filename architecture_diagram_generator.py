#!/usr/bin/env python3
"""
FinalProject 系统架构图生成器
基于当前项目的实际结构生成SVG架构图和数据流图

当前项目结构:
- Interactive_Module: CLI交互界面
- LLM_Module: 双层LLM核心 (v2.0 模块化架构)
- VLM_Module: 视觉语言模型
- Robot_Module: MCP工具注册中心
- Sim_Module: 2D仿真器
- Yolo_Module: YOLO目标检测
"""

import svgwrite
from svgwrite import cm, mm

def create_current_architecture_diagram():
    """创建当前系统的架构图"""

    dwg = svgwrite.Drawing('finalproject_current_architecture.svg', size=('1600px', '1100px'))

    # 定义样式
    styles = {
        'title': {'font_family': 'Arial', 'font_size': 24, 'font_weight': 'bold', 'text_anchor': 'middle'},
        'subtitle': {'font_family': 'Arial', 'font_size': 14, 'text_anchor': 'middle', 'fill': '#666'},
        'box_title': {'font_family': 'Arial', 'font_size': 11, 'font_weight': 'bold', 'text_anchor': 'middle'},
        'box_text': {'font_family': 'Arial', 'font_size': 9, 'text_anchor': 'middle', 'fill': '#333'},
        'arrow_text': {'font_family': 'Arial', 'font_size': 8, 'text_anchor': 'middle', 'fill': '#666'},
        'layer_title': {'font_family': 'Arial', 'font_size': 13, 'font_weight': 'bold', 'fill': '#2C3E50'},
        'code_text': {'font_family': 'monospace', 'font_size': 7, 'text_anchor': 'start', 'fill': '#555'},
    }

    # 定义颜色方案
    colors = {
        'interactive': '#E8F4FD',    # 浅蓝 - 交互层
        'llm': '#FFE4B5',            # 橙色 - LLM层
        'llm_new': '#FFD700',        # 金色 - 新LLM模块
        'vlm': '#DDA0DD',            # 紫色 - VLM层
        'robot': '#87CEEB',          # 天蓝 - Robot层
        'sim': '#F0E68C',            # 黄色 - 仿真层
        'yolo': '#FFB6C1',           # 粉红 - YOLO层
        'ros2': '#E8F5E9',           # 绿色 - ROS2通信
        'border': '#333',
        'arrow': '#666',
        'existing': '#90EE90',       # 浅绿 - 已实现
        'new': '#FFD700',            # 金色 - 新增v2.0
    }

    # 背景
    dwg.add(dwg.rect(insert=(0, 0), size=('1600px', '1100px'), fill='#FAFAFA'))

    # 标题
    dwg.add(dwg.text('FinalProject 系统架构 v2.0', insert=(800, 30), **styles['title']))
    dwg.add(dwg.text('模块化双层LLM + MCP + ROS2 + 自适应控制', insert=(800, 50), **styles['subtitle']))

    # ========== 图例 ==========
    legend_y = 80
    dwg.add(dwg.text('图例:', insert=(50, legend_y), **styles['box_title']))

    legend_items = [
        (colors['existing'], '✓ 已实现 (v1.0)'),
        (colors['new'], '★ 新增 (v2.0)'),
    ]

    x_offset = 120
    for color, label in legend_items:
        dwg.add(dwg.rect(insert=(x_offset, legend_y-10), size=(20, 15), fill=color, stroke=colors['border'], stroke_width=1))
        dwg.add(dwg.text(label, insert=(x_offset+25, legend_y), **styles['box_text']))
        x_offset += 150

    # ========== 主要模块架构 ==========
    y_start = 130

    # 1. Interactive_Module
    interactive_box = dwg.g()
    interactive_box.add(dwg.rect(insert=(50, y_start), size=(180, 90), rx=5,
                                  fill=colors['interactive'], stroke=colors['border'], stroke_width=2))
    interactive_box.add(dwg.text('Interactive_Module', insert=(140, y_start+20), **styles['box_title']))
    interactive_box.add(dwg.text('CLI 交互界面', insert=(140, y_start+40), **styles['box_text']))
    interactive_box.add(dwg.text('interactive.py', insert=(140, y_start+55), **styles['code_text']))
    interactive_box.add(dwg.text('✓ 已实现', insert=(140, y_start+70), **styles['box_text']))
    dwg.add(interactive_box)

    # 2. LLM_Module (v2.0 模块化架构)
    llm_main_box = dwg.g()
    llm_main_box.add(dwg.rect(insert=(280, y_start), size=(520, 90), rx=5,
                             fill=colors['llm'], stroke=colors['border'], stroke_width=2))
    llm_main_box.add(dwg.text('LLM_Module v2.0 (模块化架构)', insert=(540, y_start+20), **styles['box_title']))

    # LLM子模块
    llm_modules = [
        (300, y_start+35, 'high_level_llm.py', '★ 高层LLM - 任务规划器'),
        (440, y_start+35, 'low_level_llm.py', '★ 低层LLM - 执行控制器'),
        (580, y_start+35, 'task_queue.py', '★ 任务队列管理'),
        (300, y_start+60, 'execution_monitor.py', '★ 执行监控器'),
        (440, y_start+60, 'adaptive_controller.py', '★ 自适应控制器'),
        (580, y_start+60, 'llm_core.py', '兼容层 (LLMAgent)'),
    ]

    for x, y, name, desc in llm_modules:
        llm_main_box.add(dwg.rect(insert=(x, y), size=(120, 20), rx=3, fill='#FFF', stroke=colors['border'], stroke_width=1))
        llm_main_box.add(dwg.text(name, insert=(x+60, y+12), **styles['code_text']))
        # 描述文字太小，用数字标记
        if '★' in desc:
            llm_main_box.add(dwg.text('★', insert=(x+10, y+12), fill=colors['new']))

    dwg.add(llm_main_box)

    # 3. VLM_Module
    vlm_box = dwg.g()
    vlm_box.add(dwg.rect(insert=(840, y_start), size=(180, 90), rx=5,
                         fill=colors['vlm'], stroke=colors['border'], stroke_width=2))
    vlm_box.add(dwg.text('VLM_Module', insert=(930, y_start+20), **styles['box_title']))
    vlm_box.add(dwg.text('视觉语言模型', insert=(930, y_start+40), **styles['box_text']))
    vlm_box.add(dwg.text('vlm_core.py', insert=(930, y_start+55), **styles['code_text']))
    vlm_box.add(dwg.text('⚠ 仅颜色检测', insert=(930, y_start+70), **styles['box_text']))
    dwg.add(vlm_box)

    # 4. Robot_Module
    robot_box = dwg.g()
    robot_box.add(dwg.rect(insert=(1060, y_start), size=(200, 90), rx=5,
                           fill=colors['robot'], stroke=colors['border'], stroke_width=2))
    robot_box.add(dwg.text('Robot_Module', insert=(1160, y_start+20), **styles['box_title']))
    robot_box.add(dwg.text('MCP 工具注册中心', insert=(1160, y_start+40), **styles['box_text']))
    robot_box.add(dwg.text('skill.py + module/', insert=(1160, y_start+55), **styles['code_text']))
    robot_box.add(dwg.text('✓ 已实现', insert=(1160, y_start+70), **styles['box_text']))
    dwg.add(robot_box)

    # 5. ROS2 Communication
    ros2_box = dwg.g()
    ros2_box.add(dwg.rect(insert=(1300, y_start), size=(200, 90), rx=5,
                          fill=colors['ros2'], stroke=colors['border'], stroke_width=2))
    ros2_box.add(dwg.text('ROS2 Topics', insert=(1400, y_start+20), **styles['box_title']))
    ros2_box.add(dwg.text('ros_topic_comm.py', insert=(1400, y_start+40), **styles['code_text']))
    ros2_box.add(dwg.text('/robot/command', insert=(1400, y_start+55), **styles['code_text']))
    ros2_box.add(dwg.text('/robot/state, /enemies', insert=(1400, y_start+70), **styles['code_text']))
    dwg.add(ros2_box)

    # ========== 第二行: 仿真层 ==========
    y2 = y_start + 130

    # Sim_Module
    sim_box = dwg.g()
    sim_box.add(dwg.rect(insert=(50, y2), size=(300, 100), rx=5,
                         fill=colors['sim'], stroke=colors['border'], stroke_width=2))
    sim_box.add(dwg.text('Sim_Module (2D仿真器)', insert=(200, y2+20), **styles['box_title']))
    sim_box.add(dwg.text('simulator.py - Pygame仿真', insert=(200, y2+40), **styles['code_text']))
    sim_box.add(dwg.text('enemy_manager.py - 敌人管理', insert=(200, y2+55), **styles['code_text']))
    sim_box.add(dwg.text('✓ 已实现', insert=(200, y2+75), **styles['box_text']))
    dwg.add(sim_box)

    # Yolo_Module
    yolo_box = dwg.g()
    yolo_box.add(dwg.rect(insert=(400, y2), size=(300, 100), rx=5,
                          fill=colors['yolo'], stroke=colors['border'], stroke_width=2))
    yolo_box.add(dwg.text('Yolo_Module (目标检测)', insert=(550, y2+20), **styles['box_title']))
    yolo_box.add(dwg.text('yolo_simulator.py', insert=(550, y2+40), **styles['code_text']))
    yolo_box.add(dwg.text('yolo_publisher.py', insert=(550, y2+55), **styles['code_text']))
    yolo_box.add(dwg.text('✓ 已实现', insert=(550, y2+75), **styles['box_text']))
    dwg.add(yolo_box)

    # ========== v2.0 新功能详解 ==========
    y3 = y2 + 150

    new_features_box = dwg.g()
    new_features_box.add(dwg.rect(insert=(50, y3), size=(1450, 200), rx=5,
                                   fill='#FFFDE7', stroke=colors['new'], stroke_width=3))
    new_features_box.add(dwg.text('★ LLM_Module v2.0 新功能详解', insert=(775, y3+25), **styles['box_title']))

    # 新功能列表
    features = [
        (80, y3+55, '任务队列管理 (task_queue.py)', '• 任务状态跟踪 (PENDING/IN_PROGRESS/COMPLETED/FAILED/SKIPPED)\n• 任务重试机制 (可配置最大重试次数)\n• 动态插入任务 (支持队列前端插入)\n• 进度跟踪和摘要显示'),

        (450, y3+55, '执行监控器 (execution_monitor.py)', '• 超时检测 (可配置超时阈值)\n• 卡住检测 (位置长时间不变)\n• 振荡检测 (来回移动)\n• 传感器失效检测'),

        (820, y3+55, '自适应控制器 (adaptive_controller.py)', '• 多级重新规划 (4级策略)\n  - Level 1: 参数调整\n  - Level 2: 技能替换\n  - Level 3: 任务重排\n  - Level 4: 完全重新规划\n• 自动异常处理\n• 智能重试逻辑'),

        (1190, y3+55, '环境变化检测', '• 自动检测环境变化\n• 触发重新规划\n• 支持感知数据输入'),

        (80, y3+130, '高层LLM (high_level_llm.py)', '• 任务规划 (plan_tasks)\n• 重新规划 (replan_tasks)\n• 支持环境状态输入'),

        (450, y3+130, '低层LLM (low_level_llm.py)', '• 任务执行 (execute_task)\n• 工具选择\n• 参数生成\n• 感知数据融合'),

        (820, y3+130, '向后兼容 (llm_core.py)', '• LLMAgent 类保留\n• 内部使用新架构\n• 旧代码无需修改\n• enable_adaptive 参数启用新功能'),
    ]

    for x, y, title, desc in features:
        new_features_box.add(dwg.rect(insert=(x, y), size=(340, 55), rx=5, fill='#FFF', stroke=colors['border'], stroke_width=1))
        new_features_box.add(dwg.text(title, insert=(x+170, y+15), **styles['box_title']))

        # 多行描述
        lines = desc.split('\n')
        for i, line in enumerate(lines[:3]):  # 最多显示3行
            new_features_box.add(dwg.text(line[:35], insert=(x+10, y+30+i*12), **styles['box_text']))

    dwg.add(new_features_box)

    # ========== 使用方式 ==========
    y4 = y3 + 240

    usage_box = dwg.g()
    usage_box.add(dwg.rect(insert=(50, y4), size=(700, 180), rx=5,
                           fill='#E8F5E9', stroke=colors['border'], stroke_width=2))
    usage_box.add(dwg.text('使用方式', insert=(400, y4+25), **styles['box_title']))

    usage_examples = [
        (80, y4+45, '方式1: 兼容层（推荐）', 'from LLM_Module import LLMAgent\nagent = LLMAgent(api_key="...", prompt_path="...")\nresults = agent.run_pipeline(user_input, tools, execute_fn)'),

        (420, y4+45, '方式2: 启用自适应', 'agent = LLMAgent(api_key="...", prompt_path="...",\n                 enable_adaptive=True)  # ← 只需添加这个！\nresults = agent.run_pipeline(user_input, tools, execute_fn)'),

        (80, y4+115, '方式3: 新架构（完全控制）', 'from LLM_Module import AdaptiveController, HighLevelLLM, LowLevelLLM\ncontroller = AdaptiveController(high_level_llm=..., low_level_llm=...)\nresults = asyncio.run(controller.run(...))'),

        (420, y4+115, '方式4: 单独使用模块', 'from LLM_Module import TaskQueue, HighLevelLLM\nqueue = TaskQueue()\ntasks = high_level.plan_tasks(user_input, skills)\nqueue.set_tasks(tasks)'),
    ]

    for x, y, title, code in usage_examples:
        usage_box.add(dwg.rect(insert=(x, y), size=(300, 70), rx=3, fill='#FFF', stroke=colors['border'], stroke_width=1))
        usage_box.add(dwg.text(title, insert=(x+150, y+12), **styles['box_title']))

        # 代码（最多显示3行）
        lines = code.split('\n')[:3]
        for i, line in enumerate(lines):
            usage_box.add(dwg.text(line[:40], insert=(x+10, y+28+i*14), **styles['code_text']))

    dwg.add(usage_box)

    # ========== 待完成功能 ==========
    todo_box = dwg.g()
    todo_box.add(dwg.rect(insert=(800, y4), size=(700, 180), rx=5,
                           fill='#FFF3E0', stroke=colors['border'], stroke_width=2))
    todo_box.add(dwg.text('🔄 待完成功能（高优先级）', insert=(1150, y4+25), **styles['box_title']))

    todo_items = [
        (830, y4+50, '多传感器融合', '• 创建 Perception/ 模块\n• LiDAR + 深度相机 + IMU\n• 统一环境状态表示'),

        (1080, y4+50, 'VLM场景理解增强', '• 从颜色检测到场景理解\n• 物体识别与定位\n• 关系推理'),

        (830, y4+110, '技能库动态选择', '• 创建 Skills/ 模块\n• 技能基类和注册中心\n• 动态技能选择器'),

        (1080, y4+110, '高级技能', '• 攀爬、跳跃、交互\n• 搜索、巡逻'),
    ]

    for x, y, title, desc in todo_items:
        todo_box.add(dwg.rect(insert=(x, y), size=(220, 50), rx=3, fill='#FFF', stroke=colors['border'], stroke_width=1))
        todo_box.add(dwg.text(title, insert=(x+110, y+12), **styles['box_title']))

        lines = desc.split('\n')
        for i, line in enumerate(lines):
            todo_box.add(dwg.text(line[:25], insert=(x+10, y+25+i*10), **styles['box_text']))

    dwg.add(todo_box)

    # ========== 数据流连接线 ==========
    defs = dwg.defs
    marker = dwg.marker(insert=(10, 5), size=(10, 10), id='arrow')
    marker.add(dwg.path(d='M 0,0 L 10,5 L 0,10 L 2,5 Z', fill=colors['arrow']))
    defs.add(marker)

    # Interactive -> LLM
    dwg.add(dwg.line(start=(230, y_start+45), end=(280, y_start+45),
                     stroke=colors['arrow'], stroke_width=2, marker_end='url(#arrow)'))
    dwg.add(dwg.text('用户输入', insert=(255, y_start+35), **styles['arrow_text']))

    # LLM -> Robot
    dwg.add(dwg.line(start=(800, y_start+45), end=(1060, y_start+45),
                     stroke=colors['arrow'], stroke_width=2, marker_end='url(#arrow)'))
    dwg.add(dwg.text('MCP工具调用', insert=(930, y_start+35), **styles['arrow_text']))

    # Robot -> ROS2
    dwg.add(dwg.line(start=(1260, y_start+45), end=(1300, y_start+45),
                     stroke=colors['arrow'], stroke_width=2, marker_end='url(#arrow)'))
    dwg.add(dwg.text('JSON命令', insert=(1280, y_start+35), **styles['arrow_text']))

    # ROS2 -> Sim
    dwg.add(dwg.line(start=(1400, y_start+90), end=(200, y2),
                     stroke=colors['arrow'], stroke_width=2, marker_end='url(#arrow)'))
    dwg.add(dwg.text('/robot/command', insert=(800, y_start+120), **styles['arrow_text']))

    # 保存
    dwg.save()
    print("✓ 已生成: finalproject_current_architecture.svg")
    return dwg


def create_dataflow_diagram():
    """创建详细的数据流图"""

    dwg = svgwrite.Drawing('finalproject_dataflow.svg', size=('1600px', '1200px'))

    styles = {
        'title': {'font_family': 'Arial', 'font_size': 22, 'font_weight': 'bold', 'text_anchor': 'middle'},
        'phase_title': {'font_family': 'Arial', 'font_size': 14, 'font_weight': 'bold', 'fill': '#2C3E50'},
        'box_title': {'font_family': 'Arial', 'font_size': 11, 'font_weight': 'bold', 'text_anchor': 'middle'},
        'box_text': {'font_family': 'Arial', 'font_size': 9, 'text_anchor': 'middle', 'fill': '#333'},
        'flow_text': {'font_family': 'Arial', 'font_size': 8, 'text_anchor': 'middle', 'fill': '#666'},
        'code': {'font_family': 'monospace', 'font_size': 7, 'text_anchor': 'start', 'fill': '#333'},
        'code_text': {'font_family': 'monospace', 'font_size': 7, 'text_anchor': 'start', 'fill': '#333'},
        'new_marker': {'font_family': 'Arial', 'font_size': 14, 'text_anchor': 'middle', 'fill': '#FFD700'},
    }

    colors = {
        'user': '#E3F2FD',
        'process': '#FFF3E0',
        'process_new': '#FFFDE7',
        'data': '#E8F5E9',
        'border': '#333',
        'arrow': '#666',
        'new': '#FFD700',
    }

    # 背景
    dwg.add(dwg.rect(insert=(0, 0), size=('1600px', '1200px'), fill='#FAFAFA'))

    # 标题
    dwg.add(dwg.text('FinalProject 数据流详解 - 追击敌人示例', insert=(800, 30), **styles['title']))

    # ========== 场景: 启用自适应控制的追击流程 ==========
    y1 = 70

    # 标题
    dwg.add(dwg.text('场景: 使用自适应控制器的追击流程 (enable_adaptive=True)', insert=(50, y1), **styles['phase_title']))

    # 步骤1: 用户输入
    dwg.add(dwg.rect(insert=(50, y1+20), size=(150, 60), rx=5,
                     fill=colors['user'], stroke=colors['border'], stroke_width=1.5))
    dwg.add(dwg.text('用户输入', insert=(125, y1+40), **styles['box_title']))
    dwg.add(dwg.text('"追击敌人"', insert=(125, y1+55), **styles['code_text']))

    # 箭头
    dwg.add(dwg.line(start=(200, y1+50), end=(250, y1+50),
                     stroke=colors['arrow'], stroke_width=2, marker_end='url(#arrow)'))
    dwg.add(dwg.text('自然语言', insert=(225, y1+40), **styles['flow_text']))

    # 步骤2: Interactive Module
    dwg.add(dwg.rect(insert=(250, y1+20), size=(150, 80), rx=5,
                     fill=colors['process'], stroke=colors['border'], stroke_width=1.5))
    dwg.add(dwg.text('Interactive_Module', insert=(325, y1+40), **styles['box_title']))
    dwg.add(dwg.text('interactive.py', insert=(325, y1+55), **styles['code_text']))
    dwg.add(dwg.text('加载 enable_adaptive=True', insert=(325, y1+70), **styles['code_text']))
    dwg.add(dwg.text('创建 AdaptiveController', insert=(325, y1+85), **styles['code_text']))

    # 箭头
    dwg.add(dwg.line(start=(400, y1+60), end=(480, y1+60),
                     stroke=colors['arrow'], stroke_width=2, marker_end='url(#arrow)'))
    dwg.add(dwg.text('run_pipeline()', insert=(440, y1+50), **styles['flow_text']))

    # 步骤3: AdaptiveController (新!)
    dwg.add(dwg.rect(insert=(480, y1+10), size=(220, 100), rx=5,
                     fill=colors['process_new'], stroke=colors['new'], stroke_width=2))
    dwg.add(dwg.text('★ AdaptiveController', insert=(590, y1+30), **styles['box_title']))
    dwg.add(dwg.text('协调器:', insert=(590, y1+45), **styles['box_text']))
    dwg.add(dwg.text('• 高层LLM (规划)', insert=(590, y1+58), **styles['code_text']))
    dwg.add(dwg.text('• 低层LLM (执行)', insert=(590, y1+70), **styles['code_text']))
    dwg.add(dwg.text('• 执行监控 (检测)', insert=(590, y1+82), **styles['code_text']))
    dwg.add(dwg.text('• 任务队列 (管理)', insert=(590, y1+94), **styles['code_text']))

    # 箭头
    dwg.add(dwg.line(start=(700, y1+60), end=(750, y1+60),
                     stroke=colors['arrow'], stroke_width=2, marker_end='url(#arrow)'))
    dwg.add(dwg.text('异步执行', insert=(725, y1+50), **styles['flow_text']))

    # ========== 第二行: 高层规划 ==========
    y2 = y1 + 160

    dwg.add(dwg.text('步骤1: 高层LLM任务规划', insert=(50, y2), **styles['phase_title']))

    # HighLevelLLM
    dwg.add(dwg.rect(insert=(50, y2+20), size=(200, 80), rx=5,
                     fill=colors['process_new'], stroke=colors['new'], stroke_width=2))
    dwg.add(dwg.text('★ HighLevelLLM', insert=(150, y2+40), **styles['box_title']))
    dwg.add(dwg.text('high_level_llm.py', insert=(150, y2+55), **styles['code_text']))
    dwg.add(dwg.text('plan_tasks(user_input)', insert=(150, y2+70), **styles['code_text']))
    dwg.add(dwg.text('生成任务序列', insert=(150, y2+85), **styles['code_text']))

    # 箭头
    dwg.add(dwg.line(start=(250, y2+60), end=(300, y2+60),
                     stroke=colors['arrow'], stroke_width=2, marker_end='url(#arrow)'))
    dwg.add(dwg.text('tasks[]', insert=(275, y2+50), **styles['flow_text']))

    # TaskQueue
    dwg.add(dwg.rect(insert=(300, y2+20), size=(200, 80), rx=5,
                     fill=colors['process_new'], stroke=colors['new'], stroke_width=2))
    dwg.add(dwg.text('★ TaskQueue', insert=(400, y2+40), **styles['box_title']))
    dwg.add(dwg.text('task_queue.py', insert=(400, y2+55), **styles['code_text']))
    dwg.add(dwg.text('set_tasks(tasks)', insert=(400, y2+70), **styles['code_text']))
    dwg.add(dwg.text('管理任务状态', insert=(400, y2+85), **styles['code_text']))

    # 箭头
    dwg.add(dwg.line(start=(500, y2+60), end=(550, y2+60),
                     stroke=colors['arrow'], stroke_width=2, marker_end='url(#arrow)'))
    dwg.add(dwg.text('get_next()', insert=(525, y2+50), **styles['flow_text']))

    # 显示任务队列内容
    dwg.add(dwg.rect(insert=(550, y2+20), size=(180, 80), rx=5,
                     fill=colors['data'], stroke=colors['border'], stroke_width=1.5))
    dwg.add(dwg.text('任务列表:', insert=(640, y2+35), **styles['box_title']))
    dwg.add(dwg.text('1. 获取敌人位置', insert=(640, y2+50), **styles['code_text']))
    dwg.add(dwg.text('2. 追击最近的敌人', insert=(640, y2+65), **styles['code_text']))
    dwg.add(dwg.text('状态: PENDING', insert=(640, y2+80), **styles['code_text']))

    # ========== 第三行: 执行监控 + 低层执行 ==========
    y3 = y2 + 140

    dwg.add(dwg.text('步骤2: 执行循环（带监控）', insert=(50, y3), **styles['phase_title']))

    # ExecutionMonitor
    dwg.add(dwg.rect(insert=(50, y3+20), size=(200, 80), rx=5,
                     fill=colors['process_new'], stroke=colors['new'], stroke_width=2))
    dwg.add(dwg.text('★ ExecutionMonitor', insert=(150, y3+40), **styles['box_title']))
    dwg.add(dwg.text('execution_monitor.py', insert=(150, y3+55), **styles['code_text']))
    dwg.add(dwg.text('检测超时/卡住/振荡', insert=(150, y3+70), **styles['code_text']))
    dwg.add(dwg.text('环境变化检测', insert=(150, y3+85), **styles['code_text']))

    # 箭头
    dwg.add(dwg.line(start=(250, y3+60), end=(300, y3+60),
                     stroke=colors['arrow'], stroke_width=2, marker_end='url(#arrow)'))
    dwg.add(dwg.text('监控', insert=(275, y3+50), **styles['flow_text']))

    # LowLevelLLM
    dwg.add(dwg.rect(insert=(300, y3+20), size=(200, 80), rx=5,
                     fill=colors['process_new'], stroke=colors['new'], stroke_width=2))
    dwg.add(dwg.text('★ LowLevelLLM', insert=(400, y3+40), **styles['box_title']))
    dwg.add(dwg.text('low_level_llm.py', insert=(400, y3+55), **styles['code_text']))
    dwg.add(dwg.text('execute_task(task)', insert=(400, y3+70), **styles['code_text']))
    dwg.add(dwg.text('选择工具 + 生成参数', insert=(400, y3+85), **styles['code_text']))

    # 箭头
    dwg.add(dwg.line(start=(500, y3+60), end=(550, y3+60),
                     stroke=colors['arrow'], stroke_width=2, marker_end='url(#arrow)'))
    dwg.add(dwg.text('工具调用', insert=(525, y3+50), **styles['flow_text']))

    # Robot_Module
    dwg.add(dwg.rect(insert=(550, y3+20), size=(200, 80), rx=5,
                     fill=colors['process'], stroke=colors['border'], stroke_width=1.5))
    dwg.add(dwg.text('Robot_Module', insert=(650, y3+40), **styles['box_title']))
    dwg.add(dwg.text('MCP工具中心', insert=(650, y3+55), **styles['code_text']))
    dwg.add(dwg.text('chase.py', insert=(650, y3+70), **styles['code_text']))
    dwg.add(dwg.text('get_enemy_positions()', insert=(650, y3+85), **styles['code_text']))

    # 箭头
    dwg.add(dwg.line(start=(750, y3+60), end=(800, y3+60),
                     stroke=colors['arrow'], stroke_width=2, marker_end='url(#arrow)'))
    dwg.add(dwg.text('ROS2', insert=(775, y3+50), **styles['flow_text']))

    # ROS2 Communication
    dwg.add(dwg.rect(insert=(800, y3+20), size=(200, 80), rx=5,
                     fill=colors['data'], stroke=colors['border'], stroke_width=1.5))
    dwg.add(dwg.text('ROS2 Topics', insert=(900, y3+40), **styles['box_title']))
    dwg.add(dwg.text('/robot/enemies', insert=(900, y3+55), **styles['code_text']))
    dwg.add(dwg.text('std_msgs/String', insert=(900, y3+70), **styles['code_text']))
    dwg.add(dwg.text('JSON格式', insert=(900, y3+85), **styles['code_text']))

    # ========== 第四行: 自适应处理 ==========
    y4 = y3 + 160

    dwg.add(dwg.text('步骤3: 自适应处理（如果环境变化）', insert=(50, y4), **styles['phase_title']))

    # 检测环境变化
    detect_box = dwg.g()
    detect_box.add(dwg.rect(insert=(50, y4+20), size=(300, 100), rx=5,
                            fill='#FFEBEE', stroke='#E74C3C', stroke_width=2))
    detect_box.add(dwg.text('⚠️ 检测到环境变化', insert=(200, y4+40), **styles['box_title']))
    detect_box.add(dwg.text('执行监控器检测到:', insert=(200, y4+58), **styles['box_text']))
    detect_box.add(dwg.text('• 目标物体消失', insert=(80, y4+73), **styles['code_text']))
    detect_box.add(dwg.text('• 新障碍物出现', insert=(200, y4+73), **styles['code_text']))
    detect_box.add(dwg.text('• 传感器失效', insert=(320, y4+73), **styles['code_text']))
    detect_box.add(dwg.text('status = "requires_replanning"', insert=(200, y4+93), **styles['code_text']))
    dwg.add(detect_box)

    # 箭头
    dwg.add(dwg.line(start=(350, y4+70), end=(400, y4+70),
                     stroke='#E74C3C', stroke_width=2, stroke_dasharray='5,5', marker_end='url(#arrow)'))
    dwg.add(dwg.text('触发', insert=(375, y4+60), **styles['flow_text']))

    # 重新规划
    replan_box = dwg.g()
    replan_box.add(dwg.rect(insert=(400, y4+20), size=(350, 100), rx=5,
                            fill=colors['process_new'], stroke=colors['new'], stroke_width=2))
    replan_box.add(dwg.text('★ 自动重新规划', insert=(575, y4+40), **styles['box_title']))
    replan_box.add(dwg.text('high_level_llm.replan_tasks()', insert=(575, y4+58), **styles['code_text']))
    replan_box.add(dwg.text('分析失败原因 + 环境状态', insert=(575, y4+73), **styles['code_text']))
    replan_box.add(dwg.text('生成新任务序列', insert=(575, y4+88), **styles['code_text']))
    replan_box.add(dwg.text('插入到队列前端 (at_front=True)', insert=(575, y4+103), **styles['code_text']))
    dwg.add(replan_box)

    # 继续执行
    continue_box = dwg.g()
    continue_box.add(dwg.rect(insert=(800, y4+20), size=(300, 100), rx=5,
                              fill=colors['data'], stroke=colors['border'], stroke_width=1.5))
    continue_box.add(dwg.text('继续执行', insert=(950, y4+40), **styles['box_title']))
    continue_box.add(dwg.text('新任务:', insert=(950, y4+58), **styles['box_text']))
    continue_box.add(dwg.text('1. 搜索区域', insert=(850, y4+73), **styles['code_text']))
    continue_box.add(dwg.text('2. 探测环境', insert=(950, y4+73), **styles['code_text']))
    continue_box.add(dwg.text('3. 追击新目标', insert=(1050, y4+73), **styles['code_text']))
    continue_box.add(dwg.text('任务队列自动更新', insert=(950, y4+93), **styles['code_text']))
    dwg.add(continue_box)

    # ========== 第五行: 仿真器执行 ==========
    y5 = y4 + 180

    dwg.add(dwg.text('步骤4: 仿真器执行', insert=(50, y5), **styles['phase_title']))

    # 完整的执行流程
    sim_box = dwg.g()
    sim_box.add(dwg.rect(insert=(50, y5+20), size=(1050, 120), rx=5,
                         fill=colors['process'], stroke=colors['border'], stroke_width=2))
    sim_box.add(dwg.text('Sim_Module 仿真执行循环 (60 FPS)', insert=(575, y5+40), **styles['box_title']))

    sim_steps = [
        '1. robot.update() - 平滑移动到目标位置',
        '2. enemy_manager.update() - 更新所有敌人',
        '3. publish_robot_state(x, y, angle) - 每帧发布到 /robot/state',
        '4. process_action_queue() - 执行命令队列中的动作',
        '5. process_enemy_remove_queue() - 移除敌人后立即发布更新',
        '6. 每180帧 (3秒) → publish_enemy_positions() 发布到 /robot/enemies',
    ]

    for i, step in enumerate(sim_steps):
        sim_box.add(dwg.text(step, insert=(70, y5+60+i*15), **styles['code_text']))

    dwg.add(sim_box)

    # ========== 箭头标记定义 ==========
    defs = dwg.defs
    marker = dwg.marker(insert=(10, 5), size=(10, 10), id='arrow')
    marker.add(dwg.path(d='M 0,0 L 10,5 L 0,10 L 2,5 Z', fill=colors['arrow']))
    defs.add(marker)

    # 保存
    dwg.save()
    print("✓ 已生成: finalproject_dataflow.svg")
    return dwg


def create_module_structure_diagram():
    """创建模块结构详细图"""

    dwg = svgwrite.Drawing('finalproject_module_structure.svg', size=('1400px', '1000px'))

    styles = {
        'title': {'font_family': 'Arial', 'font_size': 22, 'font_weight': 'bold', 'text_anchor': 'middle'},
        'subtitle': {'font_family': 'Arial', 'font_size': 14, 'text_anchor': 'middle', 'fill': '#666'},
        'box_title': {'font_family': 'Arial', 'font_size': 12, 'font_weight': 'bold', 'text_anchor': 'middle'},
        'box_text': {'font_family': 'Arial', 'font_size': 9, 'text_anchor': 'middle', 'fill': '#333'},
        'file_text': {'font_family': 'monospace', 'font_size': 8, 'text_anchor': 'start', 'fill': '#333'},
        'arrow_text': {'font_family': 'Arial', 'font_size': 8, 'text_anchor': 'middle', 'fill': '#666'},
    }

    colors = {
        'module_v1': '#E8F5E9',
        'module_v2': '#FFFDE7',
        'border': '#333',
        'arrow': '#666',
        'v1_marker': '#4CAF50',
        'v2_marker': '#FFD700',
    }

    # 背景
    dwg.add(dwg.rect(insert=(0, 0), size=('1400px', '1000px'), fill='#FAFAFA'))

    # 标题
    dwg.add(dwg.text('FinalProject 模块结构详解', insert=(700, 30), **styles['title']))
    dwg.add(dwg.text('LLM_Module v2.0 模块化架构', insert=(700, 50), **styles['subtitle']))

    # ========== LLM_Module 结构 ==========
    y1 = 100

    dwg.add(dwg.rect(insert=(50, y1), size=(1300, 400), rx=5,
                     fill=colors['module_v2'], stroke=colors['v2_marker'], stroke_width=3))
    dwg.add(dwg.text('LLM_Module/ (v2.0 模块化架构)', insert=(700, y1+25), **styles['box_title']))

    # 文件列表
    files = [
        (80, y1+50, '__init__.py', '导出所有模块\n• HighLevelLLM, LowLevelLLM\n• TaskQueue, ExecutionMonitor\n• AdaptiveController, LLMAgent'),

        (350, y1+50, 'high_level_llm.py [★新增]', 'class HighLevelLLM:\n  • plan_tasks() - 任务规划\n  • replan_tasks() - 重新规划\n  • 支持环境状态输入'),

        (680, y1+50, 'low_level_llm.py [★新增]', 'class LowLevelLLM:\n  • execute_task() - 任务执行\n  • 环境变化检测\n  • 工具选择和参数生成'),

        (1010, y1+50, 'task_queue.py [★新增]', 'class TaskQueue:\n  • set_tasks() - 设置任务\n  • get_next_task() - 获取下一个\n  • mark_completed/failed() - 状态更新\n  • insert_tasks() - 动态插入任务'),

        (80, y1+180, 'execution_monitor.py [★新增]', 'class ExecutionMonitor:\n  • detect_anomaly() - 异常检测\n  • 超时/卡住/振荡检测\n  • 环境变化监控'),

        (350, y1+180, 'adaptive_controller.py [★新增]', 'class AdaptiveController:\n  • run() - 自适应控制循环\n  • 协调规划/执行/监控\n  • 自动重新规划'),

        (680, y1+180, 'llm_core.py [已重构]', 'class LLMAgent (兼容层):\n  • 内部使用新架构\n  • 保持向后兼容\n  • enable_adaptive 参数'),

        (1010, y1+180, 'prompts/', 'planning_prompt_2d.yaml\n  • 规划提示词模板\n  • 动态填充可用技能'),

        (80, y1+310, '核心类:', 'HighLevelLLM - 高层规划\nLowLevelLLM - 低层执行\nTask - 任务数据类\nTaskStatus - 状态枚举\nExecutionStatus - 执行状态枚举\nAnomaly - 异常数据类\nAdaptiveController - 控制器\nLLMAgent - 兼容层'),

        (500, y1+310, '核心功能:', '• 任务队列管理\n• 执行监控\n• 环境变化检测\n• 多级重新规划\n• 异常检测\n• 智能重试\n• 进度跟踪'),

        (950, y1+310, '使用方式:', 'from LLM_Module import LLMAgent\nagent = LLMAgent(\n  api_key="...",\n  enable_adaptive=True\n)\nresults = agent.run_pipeline(...)'),
    ]

    for x, y, title, content in files:
        dwg.add(dwg.rect(insert=(x, y), size=(250, 100), rx=3, fill='#FFF', stroke=colors['border'], stroke_width=1))
        dwg.add(dwg.text(title, insert=(x+125, y+15), **styles['box_title']))

        lines = content.split('\n')
        for i, line in enumerate(lines[:5]):
            if len(line) > 30:
                line = line[:30] + '...'
            dwg.add(dwg.text(line, insert=(x+10, y+30+i*14), **styles['file_text']))

    # ========== 对比：v1.0 vs v2.0 ==========
    y2 = y1 + 450

    dwg.add(dwg.text('版本对比: v1.0 vs v2.0', insert=(50, y2), **styles['box_title']))

    # v1.0
    dwg.add(dwg.rect(insert=(50, y2+30), size=(600, 200), rx=5,
                     fill=colors['module_v1'], stroke=colors['v1_marker'], stroke_width=2))
    dwg.add(dwg.text('v1.0 (旧架构)', insert=(350, y2+50), **styles['box_title']))

    v1_content = [
        'LLM_Module/',
        '  ├── llm_core.py (单一文件)',
        '  │   • LLMAgent 类',
        '  │   • plan_tasks() - 任务规划',
        '  │   • execute_single_task() - 执行',
        '  │   • run_pipeline() - 完整流程',
        '  └── prompts/',
        '      └── planning_prompt_2d.yaml',
        '',
        '特点:',
        '  ✓ 简单直接',
        '  ✗ 无任务队列管理',
        '  ✗ 无执行监控',
        '  ✗ 无环境变化检测',
        '  ✗ 无重新规划机制',
    ]

    for i, line in enumerate(v1_content):
        dwg.add(dwg.text(line, insert=(70, y2+70+i*14), **styles['file_text']))

    # v2.0
    dwg.add(dwg.rect(insert=(750, y2+30), size=(600, 200), rx=5,
                     fill=colors['module_v2'], stroke=colors['v2_marker'], stroke_width=2))
    dwg.add(dwg.text('v2.0 (模块化架构)', insert=(1050, y2+50), **styles['box_title']))

    v2_content = [
        'LLM_Module/',
        '  ├── high_level_llm.py [★]',
        '  ├── low_level_llm.py [★]',
        '  ├── task_queue.py [★]',
        '  ├── execution_monitor.py [★]',
        '  ├── adaptive_controller.py [★]',
        '  ├── llm_core.py (重构)',
        '  └── prompts/',
        '',
        '特点:',
        '  ✓ 职责分离 (高层vs低层)',
        '  ✓ 任务队列管理',
        '  ✓ 执行监控',
        '  ✓ 环境变化检测',
        '  ✓ 自适应重新规划',
        '  ✓ 完全向后兼容',
    ]

    for i, line in enumerate(v2_content):
        dwg.add(dwg.text(line, insert=(770, y2+70+i*14), **styles['file_text']))

    # ========== 升级路径 ==========
    y3 = y2 + 280

    dwg.add(dwg.text('升级路径 (无痛升级)', insert=(50, y3), **styles['box_title']))

    dwg.add(dwg.rect(insert=(50, y3+30), size=(1300, 80), rx=5,
                     fill='#E8F5E9', stroke=colors['v1_marker'], stroke_width=2))

    upgrade_steps = [
        '步骤1: 旧代码 (继续工作)',
        '  from LLM_Module import LLMAgent',
        '  agent = LLMAgent(api_key="...", prompt_path="...")',
        '  results = agent.run_pipeline(user_input, tools, execute_fn)',
        '',
        '步骤2: 启用新功能 (只需一个参数！)',
        '  from LLM_Module import LLMAgent',
        '  agent = LLMAgent(api_key="...", prompt_path="...", enable_adaptive=True)  # ← 只需添加这个',
        '  results = agent.run_pipeline(user_input, tools, execute_fn)  # 自动支持重新规划',
        '',
        '步骤3: 使用新架构 (完全控制)',
        '  from LLM_Module import AdaptiveController, HighLevelLLM, LowLevelLLM',
        '  controller = AdaptiveController(high_level_llm=..., low_level_llm=...)',
        '  results = asyncio.run(controller.run(...))  # 异步执行，完全控制',
    ]

    x_positions = [80, 300, 580, 800, 1000]
    for i, step in enumerate(upgrade_steps):
        dwg.add(dwg.text(step, insert=(x_positions[i % 5], y3+50 + (i // 5)*25), **styles['file_text']))

    # ========== 箭头标记 ==========
    defs = dwg.defs
    marker = dwg.marker(insert=(10, 5), size=(10, 10), id='arrow')
    marker.add(dwg.path(d='M 0,0 L 10,5 L 0,10 L 2,5 Z', fill=colors['arrow']))
    defs.add(marker)

    # 保存
    dwg.save()
    print("✓ 已生成: finalproject_module_structure.svg")
    return dwg


if __name__ == '__main__':
    print('开始生成 FinalProject 架构图...\n')

    create_current_architecture_diagram()
    create_dataflow_diagram()
    create_module_structure_diagram()

    print('\n✓ 所有架构图生成完成!')
    print('- finalproject_current_architecture.svg: 当前系统架构')
    print('- finalproject_dataflow.svg: 数据流详解')
    print('- finalproject_module_structure.svg: 模块结构详解')
    print('\n建议在浏览器中打开这些SVG文件查看')
