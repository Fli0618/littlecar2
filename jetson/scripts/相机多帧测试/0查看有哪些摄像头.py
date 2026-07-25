import os

import cv2

def get_available_cameras(limit=10):
    available_cameras = []
    for index in range(limit):
        # 创建 VideoCapture 实例
        # Windows 系统上建议加上 cv2.CAP_DSHOW 加快检测速度并防止卡死
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW) if os.name == "nt" else cv2.VideoCapture(index)
        
        # 检查是否成功打开
        if cap.isOpened():
            # 试着读取一帧，以确保设备能正常工作
            ret, frame = cap.read()
            if ret:
                available_cameras.append(index)
        cap.release()
    return available_cameras

if __name__ == "__main__":
    print("正在检测可用摄像头...")
    cameras = get_available_cameras()
    print(f"可用的摄像头索引列表: {cameras}")
