# 物流小车 STM32F407ZGT6 下位机工程

## 1. 项目说明

仓库地址：https://github.com/Tuzfucius/littlecar2
本工程基于 STM32F407ZGT6 和 HAL 库，用于管理底盘电机、总线舵机、WIT IMU、OPS 定位系统，以及 PC / Jetson 上位机通信。
我们使用有4个步进电机来控制底盘的前进后退等移动运动。由三个舵机来控制一些旋转的运动，包括夹爪的开合物料盘的旋转以及机械臂的旋转。

当前 README.md 所在目录是 Keil 工程文件 `4-29.uvprojx` 所在的 `MDK-ARM` 目录。上层目录为 `4-29`，是整个工程目录，主要包含：

| 目录 | 作用 |
| --- | --- |
| `Core/` | STM32 下位机业务代码、HAL 回调分发和模块接口 |
| `Drivers/` | STM32 HAL / CMSIS 驱动 |
| `MDK-ARM/` | Keil 工程文件、EIDE 配置和下位机文档入口 |
| `jetson/` | Jetson Orin Nano 或 Windows PC 侧上位机代码 |

下位机详细设计资料统一放在 `docs/` 目录下。每个源码目录也应保留对应 `README.md`，用于说明该目录下代码的职责边界。

## 2. 命名规范

当前工程采用模块前缀命名，后续新增文件、类型、宏和函数时应优先遵守以下规则：

| 前缀 | 使用范围 | 当前示例 |
| --- | --- | --- |
| `sensor_` | 传感器接入、解析和数据缓存 | `sensor_wit.*`、`sensor_ops.*` |
| `drive_` | 底层执行器、驱动协议和设备控制 | `drive_emm.*`、`drive_bus_servo.*` |
| `advance_` | 高级运动、组合动作、坐标系和业务能力封装 | `advance_chassis.*`、`advance_world.*` |
| `car_` | 车辆自身状态、属性和全局数据视图 | `car_pose.*` |

约束：

- 新增 `.c/.h` 文件应按上述前缀命名，不再使用旧的 `chassis_motion`、`bus_servo`、`host_rx`、`wit_imu`、`ops_sensor` 等文件名。
- 对外 API 可以保留硬件或设备语义清晰的函数名，例如 `Chassis_*`、`WIT_*`、`OPS_*`、`BusServo_*`；文件名和模块归属必须按前缀分类。
- 宏、状态枚举和内部辅助函数应尽量跟随所属模块前缀，避免跨模块出现同名状态或含义不清的全局符号。
- 文档、工程清单和代码引用应同步使用新文件名，避免新旧命名混用。

## 3. 当前外设分配

| 外设 | 模式 | TX | RX | 当前用途 | 对应模块 |
| --- | --- | --- | --- | --- | --- |
| USART1 | Asynchronous | PA9 | PA10 | `ONLINE_DEBUG_MODE=0` 时为 printf 调试输出，`=1` 时为在线调参 | `main.c` / `comm_tuner.*` |
| USART2 | Asynchronous | PA2 | PA3 | WIT / HWT905 IMU | `sensor_wit.*` |
| USART3 | Asynchronous | PB10 | PB11 | 张大头 Emm_V5 步进闭环电机 | `drive_emm.*` |
| UART4 | Asynchronous | PA0 | PA1 | 总线舵机 | `drive_bus_servo.*` |
| UART5 | Asynchronous | PC12 | PD2 | OPS 定位系统 | `sensor_ops.*` |
| USART6 | Asynchronous | PC6 | PC7 | Jetson 连续视觉服务 | `comm_jetson.*` |

对于电机和舵机，我们通过id编号进行配置
- 步进电机
    - 1 左前
    - 2 左后
    - 3 右前
    - 4 右后
        - 从上向下看
        - 1 3
        - 2 4
        - 从下向上看
        - 3 1
        - 4 2
    - 5 控制丝杆上下运动
    - 6 控制机械臂前后平移
- 舵机
    - 1 机械臂旋转
    - 2 机械臂抓取
    - 3 料盘旋转（示例：BusServo_SetPositionEx(3, 0, 4096, 0); 写 0 表示不限制）
    - 一圈4096脉冲

## 4. 模块说明

### 4.1 步进电机驱动

- 文件：`Core/Inc/drive_emm.h`、`Core/Src/drive_emm.c`
- 用途：张大头 Emm_V5 步进闭环电机底层协议。
- 串口：`USART3`
- 发送：使用 `HAL_UART_Transmit_DMA`，工程中已配置 `USART3 TX -> DMA1_Stream3`。
- 典型接口：`drive_emm_Vel_Control()`、`drive_emm_Pos_Control()`、`drive_emm_MonitorMotor()`、`drive_emm_IsMotorReached()`。

### 4.2 总线舵机驱动

- 文件：`Core/Inc/drive_bus_servo.h`、`Core/Src/drive_bus_servo.c`
- 用途：总线舵机设备层控制。
- 串口：`UART4`
- 当前边界：仅提供总线舵机位置命令的组帧与发送接口，不接收或解析舵机回包，不提供实际位置反馈与到位判断；`advance_arm` 忽略夹爪发送结果并固定等待 1000 ms。
- 典型接口：`BusServo_Init()`、`BusServo_SetPosition()`、`BusServo_SetPositionEx()`、`BusServo_SendGroup()`。

### 4.3 WIT IMU 传感器

- 文件：`Core/Inc/sensor_wit.h`、`Core/Src/sensor_wit.c`
- 用途：WIT / HWT905 IMU 接收、找帧、校验和数据缓存。
- 串口：`USART2`
- 接收：`DMA + UART 空闲中断`
- 当前解析：`0x51` 加速度、`0x52` 角速度、`0x53` 姿态角三类标准帧。
- 对外数据：`accel_g`、`gyro_dps`、`angle_deg`，每组三轴数据带 `valid` 和 `updated_tick`。
- 典型接口：`WIT_Init()`、`WIT_Update()`、`WIT_OnUartRxEvent()`、`WIT_GetData()`。

### 4.4 OPS 定位传感器

- 文件：`Core/Inc/sensor_ops.h`、`Core/Src/sensor_ops.c`
- 用途：OPS 全方位定位系统上行位姿帧接收和缓存。
- 串口：`UART5`
- 接收：单字节中断。
- 当前边界：只解析 OPS 上行位姿数据帧，不转发到 Jetson，不下发 OPS 配置命令。
- 对外数据：`zangle_deg`、`xangle_deg`、`yangle_deg`、`pos_x_mm`、`pos_y_mm`、`w_z_dps`。
- 典型接口：`OPS_Init()`、`OPS_Update()`、`OPS_OnByteReceived()`、`OPS_GetPose()`、`OPS_GetPoseRef()`。

### 4.5 底盘高级运动

- 文件：`Core/Inc/advance_chassis.h`、`Core/Src/advance_chassis.c`
- 用途：基于 `drive_emm` 多电机同步命令实现麦克纳姆轮底盘动作。
- 当前能力：前进、后退、左右平移、左右旋转、差速转向、四轮 RPM 直接控制、三轴麦克纳姆速度合成。
- 配置位置：四个电机 ID、方向修正、默认 RPM、默认加速度和预设动作参数集中在 `advance_chassis.h`。
- 典型接口：`Chassis_Enable()`、`Chassis_Stop()`、`Chassis_SetMotorRPMEx()`、`Chassis_MoveMecanumEx()`。
- 底盘控制权由 `advance_control` 统一维护：`NONE` 表示无控制器输出，`WORLD` 由 `advance_motion` 使用，`VISUAL` 为后续视觉定位预留。活动模式不能直接互切，必须先切换到 `NONE`，释放时会入队停止命令。

`AdvanceControl_Init()`、`AdvanceControl_SetMode()` 和 `AdvanceControl_GetMode()` 是该模块的完整公共接口。模块只维护一个 `volatile` 模式变量，不创建任务队列或通用状态机。

### 4.6 全局坐标系

- 文件：`Core/Inc/advance_world.h`、`Core/Src/advance_world.c`
- 用途：维护工程统一的 world 坐标系，完成 OPS 原始坐标到 world 位姿的转换，并提供 world/base 速度变换。
- 坐标定义：`world +Y` 为小车初始车头方向，`world +X` 为初始右侧，`yaw=0` 朝 `world +Y`，yaw 逆时针为正。
- 角度方向：俯视小车时，逆时针旋转为角度增大。若 OPS 或 WIT 因安装方向导致角度增减相反，在 `advance_world.h` 中将 `ADVANCE_WORLD_OPS_YAW_REVERSED` 或 `ADVANCE_WORLD_WIT_YAW_REVERSED` 改为 `1`。
- 初始化：OPS / WIT 掉电后会保留自身历史状态，软件不假设其上电为零；OPS 静止初始化后调用 `AdvanceWorld_ResetOrigin()`，将当前 OPS 位置、OPS 航向和 WIT yaw 记录为本次 world 坐标系零点。
- 典型接口：`AdvanceWorld_Init()`、`AdvanceWorld_Update()`、`AdvanceWorld_GetPoseCopy()`、`AdvanceWorld_WorldToBodyVelocity()`。
- `AdvanceWorld_Update()` 仅由 TIM6 调用并更新缓存；业务层通过 `AdvanceWorld_GetPoseCopy()` 读取缓存，不在读取接口内重复解析 OPS/WIT。

### 4.7 车辆状态视图

- 文件：`Core/Inc/car_pose.h`、`Core/Src/car_pose.c`
- 用途：汇总车辆自身位姿相关数据指针，作为上层读取 IMU 和 OPS 数据的统一入口。
- 当前数据：`carpose_imu` 指向 WIT 数据，`carpose_ops` 指向 OPS 位姿数据。
- 典型接口：`CarPose_Init()`。

### 4.8 Jetson 连续视觉通信

STM32 是任务主控，Jetson 是 USART6 上的视觉服务端。STM32 使用 `comm_jetson` 发送 START/STOP，Jetson 以 START 的周期持续上报最新检测结果；默认周期为 `DETECT_DEFAULT_PERIOD_MS=40 ms`。USART6 保持既有 DMA + IDLE 配置，不修改 CubeMX。

`main.c` 不再维护通用视觉阶段机。USART6 回调只缓存帧结果，TIM6 推进二维码等待与超时；协议、字段、状态码和 SESSION 过滤见 [通信协议](docs/上下位机通信协议.md)。

- 文件：`Core/Inc/comm_jetson.h`、`Core/Src/comm_jetson.c`
- 用途：STM32 通过 USART6 启动或停止 Jetson 视觉任务，并缓存当前会话的最新颜色、数字圆环、盘中心或二维码结果。
- 链路：`USART6`，DMA Circular + UART IDLE 接收，DMA 发送；帧格式为 `5A A5 CMD SESSION LEN PAYLOAD CRC16_L CRC16_H`，CRC16-Modbus 覆盖 CMD、SESSION、LEN 和 Payload。
- 检测模式：`detect_color_start()`、`detect_circle_start()`、`detect_disk_center_start()`、`detect_qr_start()`；`detect_stop()` 只停止当前检测，Jetson 保持运行并等待下一条 START。
- 默认周期：`DETECT_DEFAULT_PERIOD_MS`，默认 40 ms。每次 START 递增 SESSION，模块丢弃旧会话、错误 CRC 和模式不匹配的数据。
- 数据边界：仅缓存最新结果，`detect_get_targets()` 与 `detect_get_disk_center()` 成功读取未消费的新数据时返回 1；最多缓存 8 个目标。目标包含模型 `class_id`、像素中心、置信度、`measured` 与 `support_count`；盘中心包含状态、坐标、支持点数和 `measured_count`。
- 二维码：`detect_qr_read_blocking()` 启动一次 QR 会话并等待固定 15 字节 ASCII 任务码；等待由 `CommJetson_Update()` 在 TIM6 中推进，超时为本地 2000 ms，不在 Blocking 循环中主动轮询 UART。
- 比赛启动：Jetson 使用 `0x10`、session `0`、空 Payload 发送启动请求。`CommJetson_TakeCompetitionStart()` 消费该请求，`main.c` 每次上电仅调用一次 `App_RunTask()`；该命令不参与视觉 session 校验。
- 安全规则：盘中心零支持点固定为无目标和 `(0, 0)`，业务层不得仅凭预测结果判定到达。
- 回调与调度接口：`CommJetson_Init()`、`CommJetson_Update()`、`CommJetson_OnUartRxEvent()`、`CommJetson_OnUartError()` 和 `CommJetson_OnUartTxComplete()` 仅供 `main.c` 的 USART6 初始化、TIM6 调度和 HAL 回调分发使用。

### 4.9 机械臂双轴归零与绝对位置控制

- 升降轴使用 ID 5，顶部光电限位为绝对坐标零点，向下为正方向；滑台轴使用 ID 6，后部光电限位为零点，向前为正方向。
- `AdvanceArm_Init()` 只复位软件归零状态并注册 ID 5、6 的反馈监测。TIM6 启动后持续调用 `drive_emm_Update()`，因此阻塞归零与位置运动只能在 TIM6 已启动后执行。
- `AdvanceArm_HomeBlocking()` 固定先归零升降轴，再归零滑台轴。轴已经压住零点限位时，先确认反向运动对应的对侧限位未触发，再低速反向释放；释放过程中对侧限位触发会立即停止并使本轴归零失败。随后向零点方向搜索并进行 10 ms 二次确认；停止并发送当前位置清零命令后，才设置对应轴的归零状态。
- `AdvanceArm_MoveLiftToBlocking(position_pulse)` 与 `AdvanceArm_MoveSlideToBlocking(position_pulse)` 只接收相对零点的绝对脉冲坐标。调用前必须完成对应轴归零，且目标不得超过各轴 `*_POS_MAX`。
- 绝对位置命令使用固定正坐标方向；限位保护依据目标坐标和当前位置反馈判断实际运动方向，仅监测该方向对应的一个限位。触发限位时停止对应轴并返回 `ADVANCE_ARM_MOVE_LIMIT_REACHED`。
- 反馈失效、堵转、故障或运动超时时，停止对应轴并清除该轴归零状态；普通 `AdvanceArm_Stop()` 不清除归零状态，`AdvanceArm_EStop()` 清除两个轴的归零状态。
- 业务坐标宏保留原有分组与顺序，当前为 `0U` 占位。必须完成实机标定后才能用于取放等组合动作。

## 5. 主循环与回调边界

- `main.c` 负责初始化、顺序业务入口、TIM6 周期入口和 HAL 回调分发；最外层循环只执行 `__WFI()`。
- `HAL_UARTEx_RxEventCallback()` 中只分发 DMA / IDLE 接收事件，不直接执行业务动作。
- `HAL_UART_RxCpltCallback()` 仅分发仍使用单字节中断接收的模块，目前为 OPS。
- `HAL_UART_ErrorCallback()` 按串口来源调用对应接收模块的错误处理函数并重启接收；UART4 总线舵机当前为纯发送，不参与该回调链路。
- `HAL_UART_AbortTransmitCpltCallback()` 仅分发 USART3 电机 DMA 发送中止完成事件；USART6 发送完成仍由 `CommJetson_OnUartTxComplete()`处理。

## 6. 调试边界

- `Core/Src/main.c` 中的 `ONLINE_DEBUG_MODE` 统一控制调试模式：设为 `0` 时通过 USART1 输出 `printf` 并运行比赛主流程；设为 `1` 时禁用 `printf`，初始化 USART1 在线调参并不运行比赛主流程。
- USART6 由 `comm_jetson` 在初始化时启动 DMA + IDLE 接收；Jetson 视觉协议不使用 USART1 调试串口，也不控制底盘参数。

## 7. 世界速度与方向配置

底盘现在保留原有 RPM 调试接口，并新增物理速度接口：

- `Chassis_SetBodyVelocityEx(vx_right_mm_s, vy_forward_mm_s, wz_ccw_deg_s, acc)`
- `AdvanceMotion_SetWorldVelocityEx(vx_world_mm_s, vy_world_mm_s, wz_ccw_deg_s, acc)`

方向约定：

- `vx_right_mm_s > 0`：车体向右。
- `vy_forward_mm_s > 0`：车体向前。
- `wz_ccw_deg_s > 0`：俯视逆时针旋转。

若实车方向与约定相反，优先调整 `Core/Inc/advance_chassis.h` 中的编译期宏：

- `CHASSIS_MOTOR_*_SIGN`：单个电机正反方向。
- `CHASSIS_BODY_X_SIGN`：整体右移方向。
- `CHASSIS_BODY_Y_SIGN`：整体前进方向。
- `CHASSIS_BODY_WZ_SIGN`：整体逆时针旋转方向。
- `ADVANCE_WORLD_OPS_YAW_REVERSED` / `ADVANCE_WORLD_WIT_YAW_REVERSED`：OPS / WIT yaw 读数方向。

实车调试建议先低速确认：单轮 ID、前进、右移、逆时针旋转，再验证 world `+Y` 在不同 yaw 下方向保持一致。

## 8. GotoPose 异步目标点控制

`advance_motion` 提供 world 坐标下的本地异步目标点控制。当前仓库没有接入 `CHASSIS_GOTO_POSE` 或 `CHASSIS_GET_MOTION_STATUS` 上位机协议，顺序业务直接调用 Blocking 接口。

可用接口：

- `AdvanceMotion_GotoPoseEx(const WorldGoalPose2D_t *goal, uint8_t acc)`：接收目标点并指定 Emm 加速度参数。
- `AdvanceMotion_GotoPoseBlocking(const WorldGoalPose2D_t *goal, uint8_t acc)`：启动目标后仅等待状态终止，等待期间只执行 `__WFI()`。
- `AdvanceMotion_Update()`：由 TIM6 按 `ADVANCE_MOTION_CONTROL_PERIOD_MS` 推进一次目标点控制。
- `AdvanceMotion_Cancel()`：取消当前目标、释放 `WORLD` 控制权并停车。
- `AdvanceMotion_GetStatus()`：读取状态、当前位姿、误差和活动目标摘要。

当前 `comm_jetson` 只实现 Jetson 视觉协议；运动目标由 STM32 本地 C 接口启动，不通过当前 Jetson 通信层传输。

当前 `main.c` 直接在 TIM6 周期入口调度 `AdvanceMotion_Update()`；主循环不参与周期调度，只执行 `__WFI()`。坐标系、速度变换、GotoPose 运算逻辑和协议字段详见 `docs/坐标系与GotoPose使用说明.md` 与 `docs/上下位机通信协议.md`。

## 9. 闭环、安全与通信保护

TIM6 以 1 ms 为唯一周期入口：每 1 ms 推进 Jetson 通信状态，每 10 ms 更新 OPS/WIT、world 位姿以及 `drive_emm_Update()`，每 20 ms 按控制权执行 `AdvanceMotion_Update()`。电机通信维护与底盘闭环使用独立周期，提升 DMA/反馈超时处理及时性，同时不改变既有 20 ms PID 采样周期。主循环只运行顺序业务和 `__WFI()`。

- `GotoPose` 在进入位置和角度容差时立即下发零速度，连续稳定 `ADVANCE_MOTION_ARRIVE_HOLD_MS` 后才进入 `ARRIVED`，终态时释放 `WORLD` 控制权。
- `AdvanceControl_SetMode(NONE)` 会入队底盘停止命令；`WORLD` 与 `VISUAL` 不能同时持有控制权。
- `AdvanceMotion_Update()` 每个控制周期只消费一次已验证的 world 位姿缓存；不在 TIM6 路径中执行阻塞 UART、`HAL_Delay()` 或等待 DMA。
- USART3 使用 DMA 发送队列和 DMA/IDLE 接收。`drive_emm_Update()`在 TIM6 中推进反馈查询；发送超时使用异步 abort，不在中断内阻塞。
- 机械臂双步进轴使用固定编译期参数与独立归零状态。归零和绝对位置运动检查限位、反馈、堵转、故障和超时；夹爪仍采用固定等待的总线舵机控制。
- 驱动器心跳保护默认写为 `500 ms`，配置见 `drive_emm.h`。首次上板必须确认实际 Emm 固件支持该参数，且周期反馈查询会被驱动器视为有效心跳。
- ACK 与状态数据使用 UART 中断发送队列；UART/DMA 回调仅负责接收、入队或释放发送槽位，不直接执行业务控制。

关键配置及实车验证步骤见：

- `docs/下位机闭环与安全修复说明.md`
- `docs/坐标系与GotoPose使用说明.md`
- `docs/底盘运动控制说明.md`

## 10. 比赛事项
 - 物料颜色：初赛物料颜色包括红色、黄色、蓝色、绿色、黑色、浅蓝等，每种颜色两个（现场比赛的物料可能会有一定色差，在一定范围内变化，参赛队应适应这种变化），编号如下：红色 1、黄色 2、蓝色 3、绿色 4、黑色 5、浅蓝 6
 - 任务码格式为“颜色顺序+放置位置+颜色顺序+放置位置”。其中，第1、3组表示两批物料的颜色及搬运顺序，第2、4组表示对应物料的放置圆环编号。例如“156+123+516+231”表示：第一批按红、黑、浅蓝顺序搬运，放到1、2、3号位；第二批按黑、红、浅蓝顺序搬运，放到2、3、1号位。
 - 任务流程（要求小车一键启动，中途不得干预）
    1. 读取任务码
    机器人移动至二维码板前，识别二维码，获取两批物料的颜色顺序和放置位置，并在显示装置上显示任务码。
    2. 抓取第一批物料
    机器人前往原料区，按任务码顺序依次抓取3个物料。每次只能抓取1个，放到车上后才能抓取下一个。
    3. 运送至粗加工区
    携带1～3个物料前往粗加工区，按任务码顺序放入对应圆环位置。
    4. 转运至暂存区
    第一批物料全部放入粗加工区后，再按规定顺序逐个抓取，运送至暂存区并放入对应圆环。
    5. 抓取第二批物料
    返回原料区，按任务码顺序抓取第二批3个物料，并依次运送至粗加工区。
    6. 完成码垛
    将第二批物料从粗加工区运至暂存区，按颜色对应码放在第一批物料上，要求颜色一致、底层物料位置正确且码垛稳定。
    7. 返回并显示结果
    完成全部任务后，机器人返回指定启停区，显示正确抓取数量和正确放置数量。
 - 场地
    1. 场地整体
    比赛场地为 2400 mm × 2400 mm 的正方形平面区域。
    2. 灰色行车道
    场地中央设置十字形灰色车道，横向和纵向车道宽度均为 400 mm，交点位于场地中心。
    3. 淡黄色区域
    十字车道四周对称布置4个淡黄色区域，每个区域尺寸为 450 mm × 450 mm，机器人不得驶入。
    4. 启停区
    场地右上角和右下角各设置1个蓝色启停区，每个启停区尺寸为 300 mm × 300 mm。
    5. 原料区
    原料区位于场地上边界中部，采用直径 300 mm 的圆形电动转盘，总高度为 80～100 mm。转盘横向位置可在场地中部约 1100～1300 mm 的范围内调整，图示伸入场地约 75～85 mm。
    6. 暂存区
    暂存区沿场地左边界设置，尺寸为 580 mm × 150 mm。区域内纵向排列3个物料放置位置，首尾位置中心间距约 300 mm。
    7. 粗加工区
    粗加工区沿场地下边界设置，尺寸为 580 mm × 150 mm。区域内横向排列3个物料放置位置，首尾位置中心间距约 300 mm。
    8. 二维码板
    二维码板垂直安装在场地右侧内边缘，纵向位置位于距上边界约 1100～1300 mm 的范围内。二维码板为横向放置的A4尺寸，二维码本体为 80 mm × 80 mm。
    9. 模拟障碍物
    灰色车道上随机设置黑色圆柱形障碍物，尺寸为 φ50 mm × 100 mm，数量和位置由现场抽签确定。
    10. 允许误差
        各区域和装置的实际位置可能存在一定偏差，机器人应具备现场定位和位置修正能力。
