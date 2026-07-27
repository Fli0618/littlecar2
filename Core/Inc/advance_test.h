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

/**
 * @brief 测试底盘电机正反转方向是否与底盘约定一致。
 */
void Test_Chasis_sign(void);

/**
 * @brief 测试 Chassis_SetBodyVelocityEx 的常用车体速度组合。
 */
void Test_Chasis_SetBodyVelocityEx(void);

/**
 * @brief 测试 Chassis_MoveMecanumEx 的常用麦克纳姆轮速度组合。
 */
void Test_Chasis_MoveMecanumEx(void);

/**
 * @brief 测试底盘使能、平滑停车、立即停车及运动命令状态。
 */
void Test_Chasis_EnableAndStop(void);

/**
 * @brief 测试 Chassis_SetMotorRPMEx 的差速轮速与限速特殊用例。
 */
void Test_Chasis_SetMotorRPMExSpecial(void);

/**
 * @brief 测试 Chassis_SetBodyVelocityEx 的反向组合与零速度特殊用例。
 */
void Test_Chasis_SetBodyVelocityExSpecial(void);

/**
 * @brief 测试 Chassis_MoveMecanumEx 的原地旋转与三自由度组合用例。
 */
void Test_Chasis_MoveMecanumExSpecial(void);

#ifdef __cplusplus
}
#endif

#endif
