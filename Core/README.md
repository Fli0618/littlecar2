# Core 目录说明

`Core` 存放当前 STM32 工程的核心源码、模块接口和 HAL 入口文件。

## 子目录

- `Inc/`：公共头文件、模块接口声明、HAL 配置头文件。
- `Src/`：主程序入口、外设初始化、中断入口、设备驱动、传感器解析和高级控制实现。

## 当前命名规则

- `sensor_*`：传感器相关模块，例如 `sensor_wit`、`sensor_ops`、`sensor_limit`。
- `drive_*`：执行器和运动驱动相关模块，例如 `drive_emm`、`drive_bus_servo`。
- `advance_*`：基于底层驱动或传感器数据封装出的高级动作、坐标系或业务能力，例如 `advance_chassis`、`advance_world`。

## 当前主要模块

- `drive_emm`：张大头 Emm_V5 步进闭环驱动协议，使用 `USART3`。
- `drive_bus_servo`：总线舵机控制模块，使用 `UART4`。
- `sensor_wit`：WIT / HWT905 IMU 解析模块，使用 `USART2`。
- `sensor_ops`：OPS 定位系统解析模块，使用 `UART5`。
- `sensor_limit`：PC0~PC3 四路光电限位读取模块；只提供原始电平读取和有效状态判断，不负责 GPIO 初始化、中断或电机控制。
- `advance_chassis`：基于 `drive_emm` 的麦克纳姆底盘高级运动接口。
- `advance_control`：维护 `NONE/WORLD/VISUAL` 单一底盘控制权。
- `advance_world`：维护 world 坐标系、全局位姿和 world/base 速度变换。
- `advance_motion`：世界速度与 `GotoPose` 异步状态机；由 `main.c` 每 20 ms 调度。
- `advance_arm`：固定 ID 和编译期动作参数的完全开环阻塞式机械臂执行器；每个原子动作直接发送命令后等待 1000 ms。
- `comm_jetson`：USART6 上的 Jetson 连续视觉通信、帧解析与最新结果缓存；不参与底盘控制。

## 闭环安全边界

- `drive_emm` 负责 USART3 DMA 发送队列、DMA/IDLE 回包解析和四个底盘电机的反馈新鲜度监督。
- `advance_world` 负责 OPS 安装补偿、位置与航向的独立时间戳，以及 WIT 航向失效时的安全失效处理。
- `sensor_limit` 统一管理升降上/下、滑台前/后四个限位的电平极性；默认高电平有效，可在 `sensor_limit.h` 中改为低电平有效。
- `advance_arm` 不读取机械臂限位或电机反馈；Pick/Place 以固定顺序完成五个 1000 ms 原子动作，不能证明机械机构实际到位。
- TIM6 是周期 Update 的唯一入口；主循环只运行一次顺序业务入口，之后以 `__WFI()` 等待中断。
- 视觉通信使用 USART6 DMA + IDLE。业务层通过 `detect_color_start()`、`detect_circle_start()`、`detect_disk_center_start()` 和 `detect_stop()` 控制服务，默认周期由 `COMM_JETSON_DEFAULT_PERIOD_MS` 配置为 40 ms。
- 详细配置、参数含义与上板验收流程见 `MDK-ARM/docs/下位机闭环与安全修复说明.md`。
