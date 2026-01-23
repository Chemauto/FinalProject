#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""列出所有窗口 - 详细版"""
from Xlib import display, X

d = display.Display()
screen = d.screen()
root = screen.root

print("=== 所有窗口详细信息 ===\n")

try:
    # 获取所有窗口
    atom = d.intern_atom('_NET_CLIENT_LIST', False)
    window_ids = root.get_full_property(atom, X.AnyPropertyType, 0)

    if not window_ids or not window_ids.value:
        print("无法获取窗口列表")
    else:
        windows = window_ids.value
        print(f"找到 {len(windows)} 个窗口\n")

        for i, w in enumerate(windows):
            try:
                window = d.create_resource_object('window', w)
                geom = window.get_geometry()

                # 获取窗口类名
                try:
                    class_hint = window.get_wm_class()
                    class_name = class_hint[0] if class_hint else "N/A"
                except:
                    class_name = "N/A"

                # 获取窗口标题
                try:
                    name = window.get_wm_name()
                    if not name:
                        net_name = window.get_full_property(
                            d.intern_atom('_NET_WM_NAME', False),
                            X.AnyPropertyType, 0
                        )
                        name = net_name.value if net_name else "N/A"
                except:
                    name = "N/A"

                # 显示所有800x600的窗口
                if geom.width == 800 and geom.height == 600:
                    print(f">>> 窗口{i} (800x600) <<<")
                    print(f"    标题: {name}")
                    print(f"    类名: {class_name}")
                    print(f"    位置: ({geom.x}, {geom.y})")
                    print()
                else:
                    # 其他窗口只显示基本信息
                    if i < 5:
                        print(f"{i}. [{name[:30]}] {class_name} pos=({geom.x}, {geom.y})")

            except Exception as e:
                print(f"{i}. [ERROR: {e}]")

except Exception as e:
    print(f"错误: {e}")
    import traceback
    traceback.print_exc()
