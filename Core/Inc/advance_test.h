#ifndef __ADVANCE_TEST_H__
#define __ADVANCE_TEST_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

// 以下头文件按照依赖顺序进行排列
// 驱动类
#include "drive_emm.h"
#include "drive_bus_servo.h"

// 通信类
#include "comm_jetson.h" // 不依赖前者

// 传感器
#include "sensor_ops.h"
#include "sensor_wit.h"
#include "sensor_limit.h"
#include "advance_world.h" // 依赖ops和wit，提供世界坐标数据

// 高级动作类（依赖于驱动和传感器）
// 整车运动
#include "advance_chassis.h" 
#include "advance_motion.h"
#include "advance_control.h" // 依赖advance_chassis
#include "advance_visual.h" // 依赖comm_jetson、advance_chassis

// 机械臂运动
#include "advance_arm.h"


/**
 * @brief 执行使用 HAL_Delay 的底盘基础方向测试。
 * @note 测试期间阻塞主循环，仅用于现场调试。
 */
void AdvanceTest_BlockingMain(void);

/**
 * @brief 执行指定丝杆电机的阻塞式基础测试。
 * @param id 目标电机 ID。
 * @param is_x true 使用 X 系列命令，false 使用普通速度与位置命令。
 */
void AdvanceTest_ScrewMotor(uint8_t id, bool is_x);

#ifdef __cplusplus
}
#endif

#endif
