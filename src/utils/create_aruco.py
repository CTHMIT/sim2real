import cv2
import numpy as np
import os

# 設定畫布與主 marker（大）參數
canvas_size = 1200
border = 30  # 外白邊
big_id = 8
big_size = canvas_size - 2 * border

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
canvas = np.ones((canvas_size, canvas_size), dtype=np.uint8) * 255

# 1. 貼大 marker（id=8）
big_marker = cv2.aruco.generateImageMarker(aruco_dict, big_id, big_size)
canvas[border : border + big_size, border : border + big_size] = big_marker

# 2. 設定小 marker參數
small_size = int(big_size * 0.18)
small_ids = [20, 21, 22, 23]
# 四角位置 (你可自行微調)
offset = int(big_size * 0.07)
positions = [
    (border + offset, border + offset),  # 左上
    (border + big_size - offset - small_size, border + offset),  # 右上
    (border + offset, border + big_size - offset - small_size),  # 左下
    (
        border + big_size - offset - small_size,
        border + big_size - offset - small_size,
    ),  # 右下
]

# 3. 貼四個小 marker
for i, (x, y) in enumerate(positions):
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, small_ids[i], small_size)
    canvas[y : y + small_size, x : x + small_size] = marker_img

# 4. 貼中間五點十字
mid_ids = [30, 31, 32, 33, 34]
mid_size = int(big_size * 0.08)
cx = cy = border + big_size // 2
spacing = int(big_size * 0.15)
mid_positions = [
    (cx - spacing, cy),  # 左
    (cx + spacing - mid_size, cy),  # 右
    (cx, cy - spacing),  # 上
    (cx, cy + spacing - mid_size),  # 下
    (cx - mid_size // 2, cy - mid_size // 2),  # 中
]
for i, (x, y) in enumerate(mid_positions):
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, mid_ids[i], mid_size)
    canvas[y : y + mid_size, x : x + mid_size] = marker_img

os.makedirs("aruco_combo", exist_ok=True)
cv2.imwrite("aruco_combo/aruco_combo_layout.png", canvas)
print("產生完成：aruco_combo/aruco_combo_layout.png")
