# PC 工具目录

## 用途

`tools/` 只存放独立于 STM32 固件和 Jetson 比赛服务的 PC 工具。当前包含 `tools/pid_tuner/`（PID 在线调参）和 `tools/hsv_tuner/`（离线 HSV 阈值调参）。

## 当前状态

PID 工具的协议和验收标准见 [PID 在线调参工具计划](../MDK-ARM/docs/PID在线调参工具计划/README.md)。HSV 调参工具使用 `jetson/assets/config/hsv_colors.json`，从仓库根目录执行 `python tools/hsv_tuner/hsv_tuner_gui.py` 启动图形界面。GUI 左侧为正方形四宫格，右侧为颜色阈值、处理采样和统计诊断页签，默认分析分辨率为 `640×480`。

启动时自动加载默认 HSV 配置，也可以导入外部配置或另存为新 JSON。ROI 推荐只修改当前颜色的前两组区间，不自动保存；保存默认配置后必须重启 Jetson 视觉服务才会生效。

## 约束

- 工具不得直接修改 `Core/`、`.ioc`、Keil 工程或 `Drivers/`。
- 协议、串口和数据模型应保持独立模块，GUI 只能调用公共客户端接口。
- 测试不得依赖真实串口、相机、模型下载或小车硬件。
- 生成的缓存、日志、`__pycache__`、`.pytest_cache` 和 `*.egg-info` 不提交到 Git。
