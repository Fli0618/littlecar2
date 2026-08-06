# littlecar2 Jetson 视觉服务

`main.py` 是由 STM32 指令驱动的单线程常驻视觉服务。服务启动时同时打开二维码相机和视觉相机；二维码模式仅使用二维码相机，颜色、圆环和圆盘中心模式仅使用视觉相机。

服务初始化完成后会打开 Tkinter 比赛窗口。窗口是普通桌面窗口，不使用独占全屏、置顶或隐藏鼠标；可最小化、最大化、调整尺寸和通过 Alt+Tab 切换。

## 配置与运行

在 `main.py` 顶部配置 `SERIAL_PORT`、`SERIAL_BAUDRATE`、`CAMERA_QR_DEVICE`、`CAMERA_VISION_DEVICE` 和 `DEFAULT_PERIOD_MS`。二维码与视觉相机分别由 `QR_FRAME_WIDTH/HEIGHT`、`VISION_FRAME_WIDTH/HEIGHT` 固定为 `640x480`；启动时服务会通过 V4L2 设置、读取驱动报告尺寸并校验首帧尺寸，任一项不符合即释放已打开资源并停止启动。摄像头使用设备路径配置，例如：

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

比赛服务和维护中的视觉测试均以 `engine` 为默认后端。Jetson 的 `.pt` / PyTorch CUDA 推理可能因 CMA 连续共享内存不足而报 `NvMapMemAlloc error 12` 或 `CUBLAS_STATUS_ALLOC_FAILED`；这不代表 CUDA 安装损坏。优先使用 TensorRT engine，不要为此重装 CUDA 或 PyTorch。

先执行无相机、无串口的只读诊断与两模型预热：

```bash
PYTHONNOUSERSITE=1 /home/jetson/miniconda3/envs/yolo_env/bin/python scripts/check_yolo_runtime.py
```

若诊断仍出现 NvMap/CMA 分配失败，请先关闭使用 `/dev/nvmap` 的桌面、浏览器或远程图形程序后重试；连续内存未恢复时重启 Jetson。常规运行必须固定使用项目 `yolo_env` 并加上 `PYTHONNOUSERSITE=1`，以避免 `~/.local` 中的 Python 包覆盖已验证的运行环境。

TensorRT engine 需要在当前 Jetson 上先生成：

```bash
PYTHONNOUSERSITE=1 /home/jetson/miniconda3/envs/yolo_env/bin/python scripts/tensorrt推理测试/export_models.py
PYTHONNOUSERSITE=1 /home/jetson/miniconda3/envs/yolo_env/bin/python scripts/tensorrt推理测试/jetson_test_vision.py
```

只有更换 Jetson、JetPack/CUDA/TensorRT 版本或模型权重时，才需要运行 `export_models.py` 重建 engine。该导出步骤会走 `.pt` CUDA 路径，可能需要在关闭桌面图形程序或重启后的低内存状态执行；生成的 engine 仅保证与当前设备和 JetPack 兼容。

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

## 相机预览页面

相机预览页底部提供“二维码相机”和“视觉相机”两个按钮。空闲预览时，按钮会直接切换当前显示的采集源；视觉任务运行时，任务仍使用对应的业务相机，用户也可以切换显示另一台相机的原始画面。

`main.py` 顶部的 `ENABLE_CAMERA_PREVIEW_UI` 是相机预览页面的唯一总开关。设为 `True` 时，比赛运行页会显示“查看相机”按钮；STM32 发出有效的视觉启动命令后，GUI 会根据 `(mode, session)` 的变化自动进入相机页面，收到停止命令后返回任务码页面。同一模式的新 session 也会重新自动进入相机页面；用户在检测期间手动返回任务码页面后，未收到新会话命令时不会被下一帧强制切回。

相机页的“返回任务码”按钮只切换 GUI 页面，不停止检测或串口通信。页面隐藏后，服务继续使用原有相机帧完成检测与结果发送，但不进行 Pillow 图像转换或 GUI 刷新。空闲状态下通过“查看相机”进入预览页时，服务每 `CAMERA_PREVIEW_PERIOD_MS` 读取一次当前选择的相机，仅显示准星和相机预览状态；不会加载模型、执行推理、发送结果或启用补光灯。

二维码和视觉相机的预览准星偏移分别由 `QR_PREVIEW_AIM_OFFSET_X_PX`、`QR_PREVIEW_AIM_OFFSET_Y_PX` 与 `VISION_PREVIEW_AIM_OFFSET_X_PX`、`VISION_PREVIEW_AIM_OFFSET_Y_PX` 配置。这些偏移只影响显示，不会修改发送给 STM32 的坐标。

STM32 接收的视觉坐标始终是原始相机帧中的像素坐标，当前协议要求视觉相机稳定输出 `640x480`。YOLO 的 `imgsz=640` 仅表示推理输入尺寸，不代表输出坐标处于 `640x640` 坐标系；运行时若相机重新协商为其他尺寸，服务会丢弃该帧，不执行检测或发送坐标结果。

`scripts/gui_preview.py` 使用合成动态画面，因此在 PC 上运行时不需要真实相机、串口、模型或 Jetson GPIO。终端快捷键如下：

- `F2`：显示场地页
- `F3`：模拟二维码检测
- `F4`：模拟颜色检测
- `F5`：模拟同心圆检测
- `F6`：模拟物料盘中心检测
- `F7`：模拟停止视觉检测并返回任务码页

安装后，视觉代码以顶层包 `vision` 导入，协议代码以顶层包 `protocol` 导入。
`src` 是源码目录，不应作为包名使用；不要运行 `python -m src.vision.advance_yolo`。

二维码命令为 `CMD_START_QR = 0x05`，结果命令为 `CMD_QR_RESULT = 0x84`。二维码结果 Payload 只包含任务码本身，必须严格为 15 个 ASCII 字节，例如 `156+123+516+231`；不包含长度、结束符、状态或换行。

比赛开始命令为 `CMD_COMPETITION_START = 0x10`。用户须先在场地页选择启停区 1 或 2，随后点击“开始比赛”才会以 session `0` 发送固定 1 字节 Payload：`0x01` 表示启停区 1，`0x02` 表示启停区 2。发送成功后界面进入运行页并开始计时，此时任务码为空属于正常行为。二维码识别仍只由 STM32 后续的 `0x05` 命令触发；确认得到合法任务码后，Jetson 继续发送 `0x84`，并在窗口中保留显示该任务码。正确抓取和正确放置统计当前固定显示 `0 / 6`，尚未接入 STM32。

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

## HSV 颜色标定

离线调参工具位于仓库根目录 `tools/hsv_tuner/`，运行 `python tools/hsv_tuner/hsv_tuner_gui.py` 启动 Tkinter 图形界面。工具默认按 Jetson 的 `640×480` 分析契约处理图片，使用与正式分类相同的中心椭圆采样；不访问相机、串口、YOLO、TensorRT 或 GPIO。正式颜色链路不使用 Hough，Hough 接口仅为历史兼容。

工具启动时自动加载 `jetson/assets/config/hsv_colors.json`，支持导入默认参数、导入外部 JSON、重载当前配置和另存为。ROI 推荐只修改当前颜色的 HSV 区间，推荐值不会自动保存。保存默认配置后，需要重启 Jetson 视觉服务，新的 HSV 参数才会进入正式 YOLO bbox + HSV 分类链路。

## YOLO+HSV 混合颜色检测

正式颜色任务默认使用 `COLOR_DETECTION_BACKEND = "yolo_hsv"`：YOLO 只负责输出物料候选框与几何位置，HSV 仅在候选框内重新判定颜色。HSV 分类不确定时默认拒绝该候选框；仅在离线诊断时可通过 `uncertain_policy="keep_yolo"` 保留 YOLO 类别。`EmptySlot`（类型 `6`）不进入 HSV 分类，保持 YOLO 结果。

混合链路不使用 Hough 圆检测。Hough 仅保留在独立的纯 HSV 检测/标定辅助功能中，不是比赛服务的正式颜色检测路径。混合结果保留原有 `type`、`center`、`bbox`、`confidence` 协议字段，并额外携带 `yolo_*`、`hsv_*` 与 `classification_source` 调试字段；这些字段不会写入 STM32 目标 Payload。物料盘中心推断仍可选择纯 `yolo` 后端，避免将 HSV 分类策略强制应用到该任务。

混合链路测试不加载模型、不访问相机或串口，使用 mock 的 `detect_color` 和 `classify_bbox_hsv` 覆盖 YOLO 几何保留、HSV 修正、拒绝/回退策略、EmptySlot、跟踪调试字段、后端选择、物料盘纯 YOLO 及 Payload 兼容性：

```powershell
conda run -n low_numpy python -m pytest tests/test_hybrid_color.py tests/test_protocol_client.py -q
```

HSV 颜色规则由 `assets/config/hsv_colors.json` 管理。当前默认启用红、黄、蓝、绿四种颜色，类型编号分别为 `0`、`1`、`2`、`3`；黑色和浅蓝色配置保留但默认禁用。配置格式、字段边界和启用颜色说明见 `assets/config/README.md`。

在 Windows 开发机上执行 `python tools/hsv_tuner/hsv_tuner_gui.py` 打开离线标定界面。调参工具统一放在仓库根目录的 `tools/hsv_tuner/`；启动窗口后选择图片、配置和输出目录，界面提供 HSV 三通道直方图、H-S 二维热力图、ROI 统计和阈值线，不会访问摄像头、串口、模型或 GPIO。运行方式和输出文件命名规则见 `tools/hsv_tuner/README.md`。

HSV 核心逻辑覆盖配置读写、Mask 构建和分类阈值的纯函数测试；在 `jetson/` 目录执行：

```powershell
python -m pytest tests/test_hsv_color.py tests/test_hsv_tuner.py -q
```
