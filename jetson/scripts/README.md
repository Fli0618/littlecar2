# scripts 目录

本目录放置独立的维护与诊断脚本，不作为 `main.py` 比赛服务的启动入口。脚本不得通过修改 `sys.path` 绕过项目包安装；先在 `jetson/` 目录执行 `pip install -e .`。

HSV 调参工具已统一移至仓库根目录的 `tools/hsv_tuner/`，本目录不再存放调参实现。工具接收图片文件或图片目录，按自然顺序浏览图片，允许用户选择 ROI、调整 HSV 阈值并保存配置或导出预览图。未传入 `--output-dir` 时不导出图片，不会访问真实摄像头、串口、模型或 GPIO。

```powershell
python tools/hsv_tuner/hsv_tuner_gui.py
```

图形界面支持原子保存配置、HSV 三通道直方图、H-S 二维热力图、ROI 统计、Mask 和四宫格预览导出。输出文件使用原始图片主名、可选颜色名和种类组成，例如 `scene_red_mask.png`。`tools/hsv_tuner/outputs/` 是运行产物，不应提交到 Git。
