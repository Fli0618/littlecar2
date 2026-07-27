# littlecar2 Jetson 视觉服务

`main.py` 是由 STM32 指令驱动的单线程常驻视觉服务。服务启动时同时打开二维码相机和视觉相机；二维码模式仅使用二维码相机，颜色、圆环和圆盘中心模式仅使用视觉相机。

服务初始化完成后会打开 Tkinter 比赛窗口。窗口是普通桌面窗口，不使用独占全屏、置顶或隐藏鼠标；可最小化、最大化、调整尺寸和通过 Alt+Tab 切换。

## 配置与运行

在 `main.py` 顶部配置 `SERIAL_PORT`、`SERIAL_BAUDRATE`、`CAMERA_QR_DEVICE`、`CAMERA_VISION_DEVICE` 和 `DEFAULT_PERIOD_MS`。摄像头使用设备路径配置，例如：

```python
CAMERA_QR_DEVICE = "/dev/video0"
CAMERA_VISION_DEVICE = "/dev/video1"
```

可先运行 `scripts/jetson上测试/找摄像头.ipynb`，根据 `device_index` 和预览图确认编号与实际摄像头位置，再填写这两个常量。

主服务还可以通过 `MODEL_BACKEND` 选择 YOLO 推理后端：

```python
MODEL_BACKEND = "pt"       # 使用 .pt
MODEL_BACKEND = "engine"   # 使用 TensorRT .engine
```

TensorRT engine 需要在当前 Jetson 上先生成：

```bash
PYTHONNOUSERSITE=1 /home/jetson/miniconda3/envs/yolo_env/bin/python scripts/tensorrt推理测试/export_models.py
PYTHONNOUSERSITE=1 /home/jetson/miniconda3/envs/yolo_env/bin/python scripts/tensorrt推理测试/jetson_test_vision.py
```

Jetson 上可使用以下无图形界面测试脚本。脚本顶部的 `CAMERA_DEVICE` 可以分别修改为目标设备路径，按 `Ctrl+C` 退出：

```bash
PYTHONNOUSERSITE=1 /home/jetson/miniconda3/envs/yolo_env/bin/python scripts/jetson上测试/jetson_test_qr.py
PYTHONNOUSERSITE=1 /home/jetson/miniconda3/envs/yolo_env/bin/python scripts/jetson上测试/jetson_test_vision.py
```

二维码脚本逐帧打印原始二维码内容、确认后的任务码和状态；视觉脚本在同一帧上依次运行圆形检测和颜色物料检测，并打印两组结果。

```bash
cd /home/jetson/Project/new_littlecar2/littlecar2/jetson

PYTHONNOUSERSITE=1 /home/jetson/miniconda3/envs/yolo_env/bin/python -m pip install -e .
PYTHONNOUSERSITE=1 /home/jetson/miniconda3/envs/yolo_env/bin/python -m pytest tests -q
PYTHONNOUSERSITE=1 /home/jetson/miniconda3/envs/yolo_env/bin/python main.py
```

可单独预览界面，不会访问串口、相机或模型：

```bash
PYTHONNOUSERSITE=1 /home/jetson/miniconda3/envs/yolo_env/bin/python scripts/gui_preview.py
```

安装后，视觉代码以顶层包 `vision` 导入，协议代码以顶层包 `protocol` 导入。
`src` 是源码目录，不应作为包名使用；不要运行 `python -m src.vision.advance_yolo`。

二维码命令为 `CMD_START_QR = 0x05`，结果命令为 `CMD_QR_RESULT = 0x84`。二维码结果 Payload 只包含任务码本身，必须严格为 15 个 ASCII 字节，例如 `156+123+516+231`；不包含长度、结束符、状态或换行。

比赛开始命令为 `CMD_COMPETITION_START = 0x10`，由用户点击“开始比赛”后以 session `0`、空 Payload 发送。发送成功后界面进入运行页并开始计时，此时任务码为空属于正常行为。二维码识别仍只由 STM32 后续的 `0x05` 命令触发；确认得到合法任务码后，Jetson 继续发送 `0x84`，并在窗口中保留显示该任务码。正确抓取和正确放置统计当前固定显示 `0 / 6`，尚未接入 STM32。

高级二维码检测在最近 5 帧中确认同一码至少 3 次，仅在首次确认、任务码变更或已消失任务码再次出现时上报一次。短暂漏检不会解除锁存，连续 5 帧未识别到合法任务码后才重新布防。

可使用以下脚本检查二维码相机和逐帧检测状态；脚本会显示实时画面及识别状态，按 `Q` 或 `Esc` 退出：

```bash
/home/jetson/miniconda3/envs/yolo_env/bin/python scripts/qr_advance_test.py
```

## 开发约束

- `src` 仅作为源码目录，不作为 Python 包名；代码中使用顶层包 `vision` 和 `protocol`，不要使用 `src.vision` 或 `src.protocol`。
- 修改导入路径或包配置后，必须在项目根目录重新执行 `pip install -e .`，并确认导入路径指向当前项目。
- 不要在业务代码或测试中通过修改 `sys.path` 临时解决导入问题；应通过标准包安装解决。
- `main.py` 是服务入口，不要把 `src/vision/advance_yolo.py` 当作独立脚本直接运行。
- 提交前至少执行导入检查和视觉/协议测试；涉及硬件的测试需要在 Jetson 设备上单独确认相机、串口和设备权限。
- 测试不得依赖真实摄像头、串口或模型下载；硬件测试应与可重复的单元测试分开。
- 每完成一个独立小任务，应检查 `git diff` 和 `git status`，只提交本次任务相关的源代码、配置、文档和测试修改，不提交 `__pycache__`、`.pytest_cache`、`*.egg-info` 等生成物。
