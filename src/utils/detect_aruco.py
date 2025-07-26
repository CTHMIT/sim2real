import cv2

# 讀取圖片
img = cv2.imread("aruco/image.png")

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

corners, ids, rejected = detector.detectMarkers(img)

# 框出被偵測到的 marker
if ids is not None:
    img_marked = cv2.aruco.drawDetectedMarkers(img.copy(), corners, ids)
    print(f"共偵測到 {len(ids)} 個 aruco marker:")
    for i, id in enumerate(ids.flatten()):
        print(f"ID: {id}, 角點: {corners[i].reshape(-1,2)}")
    # 顯示或儲存
    cv2.imwrite("aruco/test_marked.png", img_marked)
    cv2.imshow("Detected Aruco", img_marked)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("未偵測到任何 aruco marker")
