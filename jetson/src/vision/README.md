# vision 目录说明

本目录提供不保存业务流程状态的视觉函数。正式接口接收 BGR 格式的
`numpy.ndarray` 图像帧，不负责摄像头、图像读取、视频流或通信。

## 接口

- `yolo.py`：`detect_color(frame_bgr)`，识别彩色物料和 `EmptySlot`。
- `yolo.py`：`detect_circle(frame_bgr)`，识别带数字的同心圆。
- `materials.py`：`detect_disk_center(frame_bgr, color_result)`，使用最多三个检测点估算物料盘中心。
- `qr.py`：`detect_qr(frame_bgr)`，识别二维码并直接返回任务码字符串。
- `yolo.py` 会按固定权重延迟加载并缓存两个模型，调用方无需传入模型对象。

物料视觉使用 `assets/models/6color-circle-v3.pt`，同心圆视觉使用
`assets/models/circle-with-number-v3.pt`。模型在首次调用后按权重路径缓存，
当前进程内不会重复加载。

检测函数返回 `{"detections": [{"type": "Red", "center": [x, y], "confidence": score}]}`。
盘中心函数返回 `{"center": [x, y], "status": 0-3, "support_points": [[x, y], ...]}`；
`status` 为参与推断的检测点数量。
