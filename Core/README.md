# Core 目录说明

`Core` 存放当前 STM32 工程的核心源码、模块接口和 HAL 入口文件。

## 子目录

- `Inc/`：公共头文件、模块接口声明、HAL 配置头文件。
- `Src/`：主程序入口、外设初始化、中断入口、设备驱动、传感器解析和高级控制实现。

`sensor_ops.c` 使用 UART5 RX 循环 DMA 接收 OPS 数据，通过 `HAL_UARTEx_RxEventCallback()` 增量处理 DMA 缓冲区中的新字节，并沿用原有 28 字节帧解析、合法性校验和 500 ms 超时失效逻辑。不要恢复 `HAL_UART_Receive_IT()`；CubeMX 重新生成后需确认 UART5 RX DMA 仍为 Circular 且 UART5 中断保持启用。

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
- `advance_control`：维护 `NONE/WORLD/VISUAL` 单一底盘控制权，并向高级控制器返回申请结果。
- `advance_visual`：基于 Jetson 新实测目标的二维车体速度 P 控制；像素误差先按 `ADVANCE_VISUAL_CAMERA_ROTATION` 旋转到车体坐标，再应用 `ADVANCE_VISUAL_BODY_X_SIGN/Y_SIGN`、死区、P 增益和限幅；提供阻塞式对齐接口，由 `main.c` 每 20 ms 在 `VISUAL` 控制权下推进。默认相机旋转角为 0 度，定义 `ADVANCE_VISUAL_TEST` 可在初始化时执行四种正交映射自检。
- `advance_world`：维护 world 坐标系、全局位姿和 world/base 速度变换。
- `advance_motion`：世界速度、`GotoPose` 与连续路径异步状态机；`FollowPathEx()` 直接引用调用方长期持有的离散路径数组，中段以动态前视切向前馈和投影 PD 连续通过软途经点，满足末端距离及参考/实测速度条件后切换到 Goto PID 精确停车；由 `main.c` 每 20 ms 调度。组合 GOTO 的大航向误差先对准策略由 `advance_motion.h` 的默认宏决定，并可在空闲状态由调参工具临时切换。
- `drive_emm` 的 DMA 队列和反馈监督由 `main.c` 每 10 ms 调度，周期定义为 `DRIVE_EMM_UPDATE_PERIOD_MS`；该周期独立于 20 ms 底盘闭环周期。
- `advance_arm`：固定 ID 和编译期动作参数的完全开环阻塞式机械臂执行器；每个原子动作直接发送命令后等待 1000 ms。
- `comm_jetson`：USART6 上的 Jetson 连续视觉通信、帧解析与最新结果缓存；不参与底盘控制。

## 闭环安全边界

- `drive_emm` 负责 USART3 DMA 发送队列、DMA/IDLE 回包解析和四个底盘电机的反馈新鲜度监督。
- `advance_world` 负责 OPS 安装补偿、位置与航向的独立时间戳，以及 WIT 航向失效时的安全失效处理。
- `sensor_limit` 统一管理升降上/下、滑台前/后四个限位的电平极性；默认高电平有效，可在 `sensor_limit.h` 中改为低电平有效。
- `advance_arm` 不读取机械臂限位或电机反馈；Pick/Place 以固定顺序完成五个 1000 ms 原子动作，不能证明机械机构实际到位。
- TIM6 是周期 Update 的唯一入口；主循环只运行一次顺序业务入口，之后以 `__WFI()` 等待中断。
- 视觉通信使用 USART6 DMA + IDLE。业务层通过 `detect_color_start()`、`detect_circle_start()`、`detect_disk_center_start()`、`detect_qr_start()` 和 `detect_stop()` 控制服务，默认周期由 `DETECT_DEFAULT_PERIOD_MS` 配置为 40 ms。
- `advance_visual` 提供 `AdvanceVisual_AlignColorBlocking()`、`AdvanceVisual_AlignDiskCenterBlocking()` 和 `AdvanceVisual_AlignCircleBlocking()` 三个业务接口。COLOR 使用 `detect_color_start()`/`detect_get_targets()`，CIRCLE 使用 `detect_circle_start()`/`detect_get_targets()`，DISK_CENTER 使用 `detect_disk_center_start()`/`detect_get_disk_center()`；三个接口共享同一套 `AdvanceVisual_Update()`、控制权、丢失和超时处理流程。
- 详细配置、参数含义与上板验收流程见 `MDK-ARM/docs/下位机闭环与安全修复说明.md`。
