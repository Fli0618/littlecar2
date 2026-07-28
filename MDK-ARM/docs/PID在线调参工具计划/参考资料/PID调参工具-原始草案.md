可以。这个工具不要一上来就做完整 GUI，正确顺序是：**先让 STM32 能在线修改参数，再让它稳定输出数据，最后才做界面。**否则协议、控制器和界面同时开发，定位问题会非常困难。

当前范围先固定为：

```text
只调 STM32 的整车位姿外环
├── X、Y 共用：Kp_pos、Ki_pos、Kd_pos
└── yaw 独立：Kp_yaw、Ki_yaw、Kd_yaw

不调整：
├── ZDT 电机内部 PID
├── 视觉对准 P 参数
└── 机械臂相关参数
```

当前控制周期为 20 ms，PID 参数还是编译期宏，USART1 的 `printf` 是逐字节阻塞发送，这正是后续需要处理的两个基础点。

---

# 一、最终目标架构

```text
电脑 PID 调参工具
        │
        │ 无线透明串口 / USB 串口
        ▼
STM32 USART1
        │
        ▼
comm_tuner
├── 接收参数修改命令
├── 校验参数
├── 返回执行结果
└── 周期发送遥测数据
        │
        ▼
advance_motion
├── 使用运行时 PID 参数
├── 计算 X、Y、yaw 外环
└── 输出 vx、vy、wz
        │
        ▼
advance_chassis
├── 坐标转换
├── 麦轮逆运动学
└── 发送四轮目标 RPM
        │
        ▼
ZDT 内部闭环
```

第一版完成后，组员可以：

1. 连接无线串口；
2. 读取当前 PID；
3. 修改六个参数；
4. 参数立即生效，不重新烧录；
5. 查看目标位置、实际位置、误差、速度曲线；
6. 停止测试；
7. 保存一份调参结果。

---

# 阶段 0：冻结范围和通信资源

## 目标

避免开发过程中不断增加功能。

## 本阶段确定

USART1 专门用于调参：

```text
USART1：
├── 接收调参命令
└── 发送遥测数据

USART6：
└── 保持 Jetson 视觉通信
```

调参固件中关闭普通 `printf`：

```c
#define DEBUG_UART_ENABLE 0U
#define TUNER_ENABLE      1U
```

原因是当前 `printf` 会调用阻塞式 `HAL_UART_Transmit()`，如果持续打印曲线数据，会干扰 20 ms 控制周期。

## 第一版开放参数

```text
Kp_pos
Ki_pos
Kd_pos
Kp_yaw
Ki_yaw
Kd_yaw
```

暂时不开放：

```text
积分限幅
位置容差
航向容差
无进展保护
控制周期
场地边界
电机内部参数
```

这些可以显示，但不允许组员随意修改。

## 完成标准

形成一份简单约束：

```text
调参链路 = USART1
数据格式 = 二进制
控制周期 = 20 ms
首版参数 = 6 个
电机内部 PID 不动
```

---

# 阶段 1：把 PID 从宏改成运行时参数

这是整个项目最核心的第一步。

## 当前问题

目前控制器直接使用：

```c
#define ADVANCE_MOTION_KP_POS ...
#define ADVANCE_MOTION_KI_POS ...
#define ADVANCE_MOTION_KD_POS ...

#define ADVANCE_MOTION_KP_YAW ...
#define ADVANCE_MOTION_KI_YAW ...
#define ADVANCE_MOTION_KD_YAW ...
```

所以参数只能通过重新编译、烧录修改。

## 修改文件

```text
Core/Inc/advance_motion.h
Core/Src/advance_motion.c
```

## 新增结构体

```c
typedef struct
{
    float kp_pos;
    float ki_pos;
    float kd_pos;

    float kp_yaw;
    float ki_yaw;
    float kd_yaw;
} AdvanceMotion_PidConfig_t;
```

保留原来的宏，但把它们改成默认值：

```c
#define ADVANCE_MOTION_DEFAULT_KP_POS  (1.0f)
#define ADVANCE_MOTION_DEFAULT_KI_POS  (0.03f)
#define ADVANCE_MOTION_DEFAULT_KD_POS  (0.10f)

#define ADVANCE_MOTION_DEFAULT_KP_YAW  (2.0f)
#define ADVANCE_MOTION_DEFAULT_KI_YAW  (0.05f)
#define ADVANCE_MOTION_DEFAULT_KD_YAW  (0.08f)
```

内部增加：

```c
static AdvanceMotion_PidConfig_t g_pid_config;
static AdvanceMotion_PidConfig_t g_pid_pending;
static volatile uint8_t g_pid_pending_valid;
```

## 新增对外接口

```c
void AdvanceMotion_GetPidConfig(
    AdvanceMotion_PidConfig_t *config);

uint8_t AdvanceMotion_SetPidConfig(
    const AdvanceMotion_PidConfig_t *config);

void AdvanceMotion_RestoreDefaultPid(void);
```

## 参数生效方式

UART 收到新参数后，不要直接逐个修改 `g_pid_config`。

正确流程：

```text
收到完整参数帧
    ↓
校验六个参数
    ↓
写入 g_pid_pending
    ↓
设置 pending_valid
    ↓
下一次 AdvanceMotion_Update() 开始时整体替换
```

这样可以避免控制中断读到：

```text
新 Kp + 旧 Ki + 旧 Kd
```

参数切换时应清除：

```text
积分历史
上一次位置历史
实测速度历史
无进展计时
```

但不需要取消当前导航任务。更稳妥的第一版也可以规定：

> 只有底盘停止时才允许修改 PID。

这样实现更简单，测试风险也更低。

## 参数范围

第一版建议设置硬限制，例如：

```text
0 ≤ Kp_pos ≤ 10
0 ≤ Ki_pos ≤ 2
0 ≤ Kd_pos ≤ 5

0 ≤ Kp_yaw ≤ 20
0 ≤ Ki_yaw ≤ 5
0 ≤ Kd_yaw ≤ 10
```

这不是最终理论范围，只是防止误输入巨大参数。

## 完成标准

暂时不用上位机。直接在 `main.c` 中调用：

```c
AdvanceMotion_PidConfig_t config;

AdvanceMotion_GetPidConfig(&config);
config.kp_pos = 1.2f;
AdvanceMotion_SetPidConfig(&config);
```

确认：

* 编译通过；
* 参数确实生效；
* 不再需要修改宏；
* 控制器未出现数据竞争。

---

# 阶段 2：实现最小 STM32 调参协议

这一阶段先不要做曲线。

## 新增文件

```text
Core/Inc/comm_tuner.h
Core/Src/comm_tuner.c
```

不要继续往 `comm_jetson` 里塞调参功能。Jetson 通信和 PC 调参是两个不同职责。

## 最小协议

建议帧格式：

```text
帧头    版本  命令  序号  长度  Payload  CRC16
A5 5A   01    1B    1B    2B      N       2B
```

例如：

```text
A5 5A | 01 | CMD | SEQ | LEN_L LEN_H | PAYLOAD | CRC_L CRC_H
```

只实现四类命令：

```c
#define TUNER_CMD_GET_PID      0x10
#define TUNER_CMD_SET_PID      0x11
#define TUNER_CMD_STOP         0x12
#define TUNER_CMD_RESTORE_PID  0x13
```

STM32 返回：

```c
#define TUNER_RESP_ACK         0x80
#define TUNER_RESP_PID         0x81
#define TUNER_RESP_ERROR       0xFF
```

## SET_PID Payload

固定发送六个 `float`：

```c
typedef struct
{
    float kp_pos;
    float ki_pos;
    float kd_pos;
    float kp_yaw;
    float ki_yaw;
    float kd_yaw;
} Tuner_SetPidPayload_t;
```

两端统一小端字节序。

## STOP 命令

收到停止命令后调用：

```c
AdvanceMotion_CancelIfActive();
Chassis_SmoothStop(...);
```

还应清空 PID 历史。

## 接收方式

USART1 建议使用：

```text
DMA + UART IDLE
```

回调只负责：

```text
把收到的字节放入解析缓冲区
```

不要在 UART 中断中：

```text
解析完整业务
修改 PID
调用底盘运动
发送长数据
```

## 完成标准

先使用 Python 命令行脚本或十六进制串口助手完成：

```text
GET_PID → STM32 返回当前六个参数
SET_PID → STM32 返回 ACK
再次 GET_PID → 参数已经改变
RESTORE_PID → 恢复默认值
STOP → 底盘停止
```

只有这一阶段稳定后，才开始做遥测。

---

# 阶段 3：在控制器中增加调试快照

当前 `AdvanceMotion_RuntimeStatus_t` 已经包含：

```text
目标位姿
实际位姿
X/Y 误差
合成位置误差
yaw 误差
运行状态
```

但是它还没有暴露控制输出和实测速度。

## 建议增加调试快照

```c
typedef struct
{
    uint32_t tick_ms;

    float target_x_mm;
    float target_y_mm;
    float target_yaw_deg;

    float actual_x_mm;
    float actual_y_mm;
    float actual_yaw_deg;

    float error_x_mm;
    float error_y_mm;
    float error_yaw_deg;

    float command_vx_mm_s;
    float command_vy_mm_s;
    float command_wz_deg_s;

    float measured_vx_mm_s;
    float measured_vy_mm_s;
    float measured_wz_deg_s;

    float integral_x;
    float integral_y;
    float integral_yaw;

    uint8_t state;
    uint8_t linear_saturated;
    uint8_t yaw_saturated;
} AdvanceMotion_DebugSnapshot_t;
```

新增接口：

```c
uint8_t AdvanceMotion_GetDebugSnapshot(
    AdvanceMotion_DebugSnapshot_t *snapshot);
```

## 为什么必须包含这些数据

当前控制器计算形式是：

```text
P：位置误差
I：误差积分
D：负的实测速度
```

所以至少要同时观察：

```text
误差
积分
实测速度
最终速度指令
是否限幅
```

只画目标位置和实际位置，无法判断控制器为什么表现异常。

## 线程安全

因为 `AdvanceMotion_Update()` 在 TIM6 中执行，而 USART 发送可能在另一个回调中读取快照，因此复制快照时需要短临界区：

```c
__disable_irq();
*snapshot = g_debug_snapshot;
__enable_irq();
```

不要在临界区内发送串口。

## 完成标准

通过调试器或临时断点确认：

* 每个 20 ms 周期快照更新；
* X、Y、yaw 数据合理；
* 停止后速度指令为 0；
* 状态切换正确。

---

# 阶段 4：加入遥测发送

## 遥测频率

PID 控制频率是 50 Hz，但第一版遥测建议：

```text
控制：50 Hz
遥测：25 Hz
```

也就是每两个控制周期发送一次。

先不要直接 50 Hz 发送所有浮点数据，避免增加无意义负担。

## 发送方式

必须使用：

```text
USART1 TX DMA
```

不能使用阻塞式：

```c
HAL_UART_Transmit(...)
printf(...)
```

建议维护：

```text
双缓冲或小型发送队列
```

第一版也可以更简单：

```text
如果 DMA 空闲 → 发送最新快照
如果 DMA 忙 → 丢弃本次遥测
```

调参曲线允许偶尔掉一帧，不应因为遥测阻塞控制。

## 遥测消息

```c
#define TUNER_TELEMETRY_MOTION 0x01
```

建议第一版发送：

```text
tick
sequence
state

target_x/y/yaw
actual_x/y/yaw
error_x/y/yaw

command_vx/vy/wz
measured_vx/vy/wz

linear_saturated
yaw_saturated
```

积分项可以第二版再加入，避免首帧过大。

## 完成标准

使用 Python 脚本连续接收 30 秒：

* CRC 全部正确；
* 序号连续或能统计丢帧；
* STM32 控制周期没有明显抖动；
* 发送队列不会堆积；
* 停止运动后曲线归零。

---

# 阶段 5：先开发 Python 协议库和命令行工具

不要直接写 GUI。

## 目录结构

```text
tools/pid_tuner/
├── protocol.py
├── serial_client.py
├── cli.py
├── requirements.txt
└── README.md
```

## protocol.py

负责：

```text
帧编码
CRC16
字节流找帧
Payload 解包
异常帧恢复
```

## serial_client.py

负责：

```text
串口打开/关闭
后台接收线程
命令序号
等待 ACK
遥测回调
```

## cli.py

支持：

```bash
python cli.py ports
python cli.py get-pid
python cli.py set-pid --kp-pos 1.2 --ki-pos 0.03 ...
python cli.py monitor
python cli.py stop
```

## 为什么先写 CLI

CLI 能把问题限制在：

```text
STM32 固件
串口
协议
参数生效
```

如果直接做 GUI，出现故障时无法判断是：

```text
按钮没触发
线程问题
串口问题
协议问题
STM32 问题
```

## 完成标准

CLI 可以稳定完成：

* 自动寻找串口；
* 读取参数；
* 修改参数；
* 打印遥测；
* 停止运动；
* 检测 CRC 错误；
* 断线后正确退出。

---

# 阶段 6：制作最小 GUI

推荐：

```text
PySide6
pyqtgraph
pyserial
```

## 第一版界面

### 左侧：连接和参数

```text
串口
波特率
连接/断开
当前状态

位置 PID：
Kp Ki Kd

航向 PID：
Kp Ki Kd

读取
应用
恢复默认
```

### 中间：测试控制

```text
目标 X
目标 Y
目标 yaw
vmax
wmax

启动测试
停止
清空曲线
```

注意：第一版可以暂时不允许 GUI 直接启动导航，只负责调参和停止。导航目标仍然由 STM32 测试程序给出，这样更安全。

### 右侧：曲线

三个页签即可：

```text
位置：
target_x / actual_x
target_y / actual_y

航向：
target_yaw / actual_yaw

控制：
error
command velocity
measured velocity
```

不要一开始做十几个仪表盘、动画车辆和场地图。

## 线程设计

```text
GUI 主线程：
└── 界面刷新

串口线程：
├── 接收字节
├── 协议解析
└── 把数据送入线程安全队列

定时器：
└── 每 30～50 ms 刷新曲线
```

不要每收到一帧就直接刷新 Qt 曲线。

## 曲线缓冲

保留最近：

```text
30～60 秒
```

例如 25 Hz、60 秒：

```text
1500 个采样点
```

足够调 PID，不需要无限增长。

## 完成标准

* GUI 不阻塞；
* 曲线持续刷新；
* 参数修改有 ACK；
* 修改参数时在曲线上增加标记；
* 串口断开后按钮自动禁用；
* STOP 按钮始终可见。

---

# 阶段 7：加入安全保护

这一步必须在真实小车高速调参前完成。

## 1. 参数修改条件

第一版规定：

```text
只有底盘停止时才允许修改 PID
```

STM32 若检测到 `AdvanceMotion` 正在运行，返回：

```text
TUNER_ERROR_BUSY
```

后期确认安全后，再支持运动过程中切换。

## 2. 心跳保护

电脑每 500 ms 发送心跳。

STM32 若超过例如 1500 ms 没有收到心跳：

```text
取消导航
平滑停车
关闭遥测或进入安全状态
```

## 3. 参数范围

STM32 必须自行校验，不能只依赖 GUI。

包括：

```text
isfinite()
非负
最大值限制
禁止 NaN
禁止 Inf
```

## 4. 速度限制

调参模式自动限制：

```text
较低 vmax
较低 wmax
较低电机 RPM
```

调参模式不要直接使用比赛速度。

## 5. 急停

GUI 的 STOP 不是物理急停。

真实调试时还需要：

```text
机械急停
电机电源切断能力
有人在车旁观察
小车架空进行首轮测试
```

---

# 阶段 8：参数保存和版本管理

第一版参数只保存在 RAM：

```text
复位后恢复代码默认值
```

这是正确的，避免调试错误参数永久写入。

## 后续保存方案

提供三个层级：

### 临时应用

```text
SET_PID
```

只修改 RAM。

### 本地配置文件

电脑保存：

```text
configs/
├── x_axis_test.json
├── y_axis_test.json
├── yaw_test.json
└── final.json
```

### 固件默认值

调参完成后：

```text
GUI 导出 C 宏
```

例如：

```c
#define ADVANCE_MOTION_DEFAULT_KP_POS (1.25f)
#define ADVANCE_MOTION_DEFAULT_KI_POS (0.02f)
#define ADVANCE_MOTION_DEFAULT_KD_POS (0.15f)
```

然后人工审查并提交 Git。

不建议第一版写 STM32 Flash 或 EEPROM。只有确认确实需要设备掉电保存时再实现。

---

# 阶段 9：实际 PID 调试流程

工具完成后，也不能三个轴同时乱调。

## 第一步：验证系统方向

先设置：

```text
Ki = 0
Kd = 0
较小 Kp
```

确认：

* X 误差为正时，实际 X 朝目标移动；
* Y 误差为正时，实际 Y 朝目标移动；
* yaw 误差为正时，实际 yaw 朝正确方向旋转。

如果方向错了，调 PID 没有意义。

## 第二步：调 X 方向

目标只改变 X：

```text
Y 不变
yaw 不约束或固定
```

顺序：

```text
先调 Kp_pos
再加 Kd_pos
最后视情况增加少量 Ki_pos
```

观察：

```text
上升时间
超调
振荡
稳定时间
稳态误差
速度饱和
```

## 第三步：调 Y 方向

由于 X、Y 当前共用参数，要验证同一组参数在前后和横移上的效果。

如果 X、Y 动态差异非常大，再考虑后续拆成：

```text
Kp_x / Ki_x / Kd_x
Kp_y / Ki_y / Kd_y
```

现在不要提前拆。

## 第四步：调 yaw

原地旋转，不同时移动 X、Y。

顺序仍然是：

```text
Kp_yaw
Kd_yaw
Ki_yaw
```

## 第五步：联合测试

依次进行：

```text
X + Y
X + yaw
Y + yaw
X + Y + yaw
```

最终再测试比赛路径。

---

# 推荐开发顺序汇总

```text
阶段 0：固定范围和 USART1 用途
阶段 1：PID 改成运行时结构体
阶段 2：实现 GET/SET/STOP 最小协议
阶段 3：增加控制器调试快照
阶段 4：DMA 周期发送遥测
阶段 5：Python CLI 验证协议
阶段 6：PySide6 GUI 和实时曲线
阶段 7：心跳、限幅和安全停车
阶段 8：配置保存和代码导出
阶段 9：按 X、Y、yaw 顺序实车调参
```

开发时最重要的阶段门槛是：

```text
运行时参数稳定
    ↓
协议稳定
    ↓
遥测稳定
    ↓
CLI 稳定
    ↓
最后才做 GUI
```

第一轮代码编写建议只完成**阶段 1 和阶段 2**。这两阶段通过后，在线调参的核心能力实际上就已经成立了。
