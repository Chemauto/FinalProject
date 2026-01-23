#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
仿真器窗口位置获取工具

这个模块提供了多种方法来获取仿真器窗口的位置信息。
由于窗口管理器的缓存机制，实时位置更新可能有限制。

使用方法:
1. python3 simulator_position.py          # 单次检测
2. python3 simulator_position.py --test  # 实时测试
3. python3 simulator_position.py --help  # 显示帮助

推荐用法:
- 对于截图功能，使用 get_current_position() 获取当前位置
- 位置信息在窗口移动后可能需要重新获取
"""

import subprocess
import re
import time
import sys

def get_current_position():
    """获取仿真器窗口当前位置（推荐使用）"""
    try:
        # 方法1: 使用xwininfo直接查找窗口
        result = subprocess.run(['xwininfo', '-name', '追击功能测试 - 2D Robot Simulator'], 
                              capture_output=True, text=True, timeout=5)
        
        if result.returncode == 0:
            # 解析xwininfo输出
            geometry = parse_xwininfo_output(result.stdout)
            if geometry:
                return geometry
        
        # 方法2: 如果方法1失败，使用树形查找
        result = subprocess.run(['xwininfo', '-tree', '-root'], 
                              capture_output=True, text=True, timeout=5)
        
        if result.returncode == 0:
            geometry = parse_tree_output(result.stdout)
            if geometry:
                return geometry
        
        return None
    
    except Exception as e:
        print(f"获取窗口位置失败: {e}")
        return None

def parse_xwininfo_output(output):
    """解析xwininfo -name 的输出"""
    geometry = {}
    
    for line in output.split('\n'):
        line = line.strip()
        
        if 'Absolute upper-left X:' in line:
            match = re.search(r'(\d+)', line)
            if match:
                geometry['x'] = int(match.group(1))
        
        elif 'Absolute upper-left Y:' in line:
            match = re.search(r'(\d+)', line)
            if match:
                geometry['y'] = int(match.group(1))
        
        elif 'Width:' in line:
            match = re.search(r'(\d+)', line)
            if match:
                geometry['width'] = int(match.group(1))
        
        elif 'Height:' in line:
            match = re.search(r'(\d+)', line)
            if match:
                geometry['height'] = int(match.group(1))
    
    # 检查是否获取到完整信息
    if all(key in geometry for key in ['x', 'y', 'width', 'height']):
        return {
            'position': (geometry['x'], geometry['y']),
            'size': (geometry['width'], geometry['height']),
            'method': 'xwininfo_direct'
        }
    
    return None

def parse_tree_output(output):
    """解析xwininfo -tree -root 的输出"""
    lines = output.split('\n')
    
    for line in lines:
        if ('追击功能测试 - 2D Robot Simulator' in line and 
            'simulator.py' in line and '800x600' in line):
            
            # 解析窗口信息
            match = re.search(r'(\d+)x(\d+)\+(\d+)\+(\d+).*\+(\d+)\+(\d+)', line)
            if match:
                width, height, rel_x, rel_y, abs_x, abs_y = match.groups()
                
                return {
                    'position': (int(abs_x), int(abs_y)),
                    'size': (int(width), int(height)),
                    'method': 'xwininfo_tree',
                    'relative_position': (int(rel_x), int(rel_y))
                }
    
    return None

def test_position_update():
    """测试位置更新功能"""
    print("=== 仿真器窗口位置测试 ===")
    print("当前检测到的位置信息:")
    
    position = get_current_position()
    
    if position:
        print(f"位置: {position['position']}")
        print(f"尺寸: {position['size']}")
        print(f"检测方法: {position['method']}")
        
        if 'relative_position' in position:
            print(f"相对位置: {position['relative_position']}")
        
        print(f"\n推荐使用坐标: {position['position'][0]},{position['position'][1]}")
        
        print("\n注意: 由于窗口管理器缓存机制，")
        print("窗口移动后可能需要重新运行此脚本获取更新后的位置。")
        
    else:
        print("未找到仿真器窗口")
        print("\n请确保:")
        print("1. 仿真器正在运行")
        print("2. 窗口标题为 '追击功能测试 - 2D Robot Simulator'")

def show_help():
    """显示帮助信息"""
    print("=== 仿真器窗口位置获取工具 ===")
    print()
    print("功能:")
    print("  获取仿真器窗口的屏幕位置和尺寸信息")
    print()
    print("使用方法:")
    print("  python3 simulator_position.py           # 获取当前位置")
    print("  python3 simulator_position.py --test     # 测试功能")
    print("  python3 simulator_position.py --help     # 显示帮助")
    print()
    print("输出格式:")
    print("  位置: (x, y) - 窗口左上角的屏幕坐标")
    print("  尺寸: (width, height) - 窗口的宽度和高度")
    print()
    print("注意事项:")
    print("  - 窗口移动后需要重新运行脚本获取新位置")
    print("  - 位置信息用于截图时的区域定位")
    print("  - 推荐使用绝对位置坐标")

def main():
    """主函数"""
    if len(sys.argv) > 1:
        if sys.argv[1] == '--help':
            show_help()
        elif sys.argv[1] == '--test':
            test_position_update()
        else:
            print("未知参数，使用 --help 查看帮助")
    else:
        # 默认行为：获取当前位置
        position = get_current_position()
        
        if position:
            print(f"{position['position'][0]},{position['position'][1]}")
        else:
            print("ERROR:未找到窗口")
            sys.exit(1)

if __name__ == "__main__":
    main()