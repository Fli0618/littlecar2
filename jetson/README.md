# littlecar2 Jetson

## 目录

```text
jetson/
  main.py       正式流程入口
  src/vision    正式视觉接口
  src/protocol  下位机通信协议
  tests/        自动化测试
  scripts/      算法研究和验证脚本
  assets/       模型和测试图片
```

## 运行

```powershell
conda run -n low_numpy pip install -e .
conda run -n low_numpy python main.py
```

## 视觉 API

```python
from vision import (
    detect_circle,
    detect_color,
    detect_disk_center,
    detect_qr,
)

colors = detect_color(frame_bgr)
circles = detect_circle(frame_bgr)
center = detect_disk_center(frame_bgr, colors)
```

连续视频场景可使用 `advance_detect_color(frame_bgr)`、`advance_detect_circle(frame_bgr)` 和
`advance_detect_disk_center(frame_bgr)`。高级接口会按目标类型进行距离关联、卡尔曼平滑和多帧确认。
相机编号常量为 `CAMERA_INDEX_COLOR_CIRCLE = 1` 与 `CAMERA_INDEX_QR = 0`。

物料颜色、同心圆和物料盘中心定位均使用 YOLO。两个模型会在首次调用时加载一次，
之后逐帧复用。`EmptySlot` 类别会保留在检测结果中，并可参与中心定位。

二维码接口直接返回任务码字符串；未识别到二维码时返回 `None`。

## 测试

```powershell
conda run -n low_numpy python -m pytest tests -q
```

`main.py` 不会自动打开摄像头、读取图片、加载模型或发送串口命令。
