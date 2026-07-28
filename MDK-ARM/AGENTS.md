# 智能体开发约束

## 修改范围

- 当前目录为 `MDK-ARM/`，仅存放 Keil 工程文件、EIDE 配置和下位机文档入口，不是项目或 Git 仓库根目录。
- 项目根目录为上一级 `..\`（`4-29/`），其中 `Core/` 为 STM32 业务代码，`Drivers/` 为 HAL/CMSIS 驱动，`jetson/` 为 Jetson/PC 上位机代码；Git 仓库位于 `..\.git`。
- 仅阅读和修改 `Core/`、`jetson/`、`MDK-ARM/`、`tools/` 中与任务相关的内容；不修改 `Drivers/`。
- `tools/` 用于独立的 PC 工具，当前计划中的 PID 调参工具位于 `tools/pid_tuner/`，不与 Jetson 比赛服务混用运行入口。
- 不直接修改 CubeMX 硬件配置。需要调整引脚、DMA、NVIC 或串口参数时，说明应由用户在 CubeMX 中完成的改动。
- 修改 CubeMX 生成文件时，仅在预留的 `USER CODE` 区域编写用户代码。

## 模块与命名

- 保持外设驱动与业务流程分层；业务动作不得直接写入底层协议模块。
- 新增模块按职责使用前缀：`sensor_`（传感器）、`drive_`（驱动控制）、`advance_`（高级运动）、`comm_`（通信）、`car_`（车辆状态）。
- 文件名、类型、宏、内部辅助函数与所属模块前缀保持一致；对外 API 可保留清晰的硬件或设备语义。
- PC 工具使用独立 Python 包和清晰的模块边界，协议、串口客户端、数据模型、存储和 GUI 不互相绕过公共接口。

## 质量与验证

- 保持实现简洁，优先完成用户指定的最小功能，不额外引入复杂验证链路。
- 为非直观的方法、结构体和关键约束补充简短注释；模块变更时同步更新所在目录的说明文档。
- STM32 工程通常不在本环境编译；由用户使用 Keil/CubeMX 并上板验证。提交时说明未执行的验证项。
- Jetson 代码使用顶层包 `vision`、`protocol`，不使用 `src.*` 导入，也不通过修改 `sys.path` 规避包配置问题；修改包配置或导入路径后，在 `jetson/` 重新执行 `pip install -e .`。
- Jetson 的 `main.py` 是服务入口，不将 `src/vision/advance_yolo.py` 作为独立脚本运行。单元测试不得依赖真实相机、串口或模型下载；硬件测试须单独在 Jetson 上确认。
- `tools/` 中的单元测试不得依赖真实串口或车辆硬件；需要硬件的测试必须单独标记并在 Windows/现场环境执行。工具的缓存、日志、`__pycache__`、`.pytest_cache` 和 `*.egg-info` 不得提交。
- 提交前检查 `git diff` 与 `git status`，仅提交本任务相关的内容；不提交 `__pycache__`、`.pytest_cache`、`*.egg-info` 等生成物。
