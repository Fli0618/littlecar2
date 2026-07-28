# PC 工具目录

## 用途

`tools/` 只存放独立于 STM32 固件和 Jetson 比赛服务的 PC 工具。首个计划中的子项目是 `tools/pid_tuner/`，面向 Windows 11 的 PID 在线调参工具。

## 当前状态

目前只建立目录说明，尚未创建 `pid_tuner` 实现代码。具体开发顺序、协议和验收标准见 [PID 在线调参工具计划](../MDK-ARM/docs/PID在线调参工具计划/README.md)。

## 约束

- 工具不得直接修改 `Core/`、`.ioc`、Keil 工程或 `Drivers/`。
- 协议、串口和数据模型应保持独立模块，GUI 只能调用公共客户端接口。
- 测试不得依赖真实串口、相机、模型下载或小车硬件。
- 生成的缓存、日志、`__pycache__`、`.pytest_cache` 和 `*.egg-info` 不提交到 Git。
