#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLO 检测 - 手动刷新版（按s键刷新）"""
import os
os.environ['YOLO_OFFLINE'] = 'True'

import cv2
import mss
import numpy as np
from ultralytics import YOLO

# 配置
MODEL = "/home/xcj/work/FinalProject/Yolo_Module/best.pt"
MONITOR = {"top": 100, "left": 100, "width": 1000, "height": 800}

# 加载模型
model = YOLO(MODEL)
sct = mss.mss()

print(f"[YOLO] 检测区域: {MONITOR}")
print("[YOLO] 按 's' 键刷新检测，按 'q' 键退出\n")

try:
    while True:
        # 截屏
        img = np.array(sct.grab(MONITOR))[:,:,:3]

        # 检测
        result = model.predict(
            source=img,
            conf=0.5,
            save=False,
            verbose=False,
            )[0]

        # 显示检测结果
        cv2.imshow("YOLO Detection", result.plot())

        # 等待按键
        key = cv2.waitKey(0) & 0xFF

        if key == ord('q'):  # q键退出
            break

except KeyboardInterrupt:
    pass
finally:
    cv2.destroyAllWindows()
    print("[YOLO] 已退出")
