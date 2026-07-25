# Inc 目录说明

本目录存放 `Core` 层头文件。

- `main.h`：主程序公共声明。
- `stm32f4xx_hal_conf.h`：HAL 组件配置。
- `stm32f4xx_it.h`：中断入口声明。
- `drive_emm.h`：张大头 Emm_V5 步进闭环驱动接口。
- `drive_bus_servo.h`：总线舵机控制与预留位置反馈接口；实际回读协议待接入。
- `sensor_wit.h`：WIT / HWT905 IMU 数据接口。
- `sensor_ops.h`：OPS 定位系统数据接口。
- `sensor_limit.h`：PC0~PC3 光电限位读取接口、限位标识和可配置有效电平定义。
- `advance_chassis.h`：麦克纳姆底盘高级运动接口。
- `advance_world.h`：全局坐标系、world 位姿和坐标变换接口。
- `advance_arm.h`：完全开环的阻塞式机械臂接口，提供夹爪、固定抓取/放置和停止控制。
- `advance_test.h`：下位机人工联调接口，仅提供阻塞式测试。
