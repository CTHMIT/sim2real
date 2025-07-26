import cv2
import os

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
marker_id = 8
marker_size = 800  # 800x800 px

marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

os.makedirs("aruco", exist_ok=True)
cv2.imwrite("aruco/aruco_id8_800x800.png", marker_img)
print("已產生 aruco/aruco_id8_800x800.png")
