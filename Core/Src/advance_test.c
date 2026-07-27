#include "advance_test.h"

#include <stdio.h>

#include "drive_emm.h"
#include "main.h"

void AdvanceTest_BlockingMain(void)
{
  DriveEmm_MotorFeedback_t feedback;
  DriveEmm_Diagnostics_t diagnostics;
  int32_t target_pulse;
  uint16_t poll_count;
  uint8_t reached;

  printf("[TEST] drive normal control modes start\r\n");
  if (drive_emm_GetDiagnostics(&diagnostics) == HAL_OK)
  {
    printf("[TEST][DRIVE] diag q=%u/%u active=%u wait=%u monitor=%u txerr=%lu rx=%lu ack=%lu bad=%lu drop=%lu unknown=%lu timeout=%lu\r\n",
           (unsigned int)diagnostics.tx_queue_count, (unsigned int)diagnostics.tx_queue_depth,
           (unsigned int)diagnostics.tx_active, (unsigned int)diagnostics.query_waiting,
           (unsigned int)diagnostics.feedback_monitor_enabled,
           (unsigned long)diagnostics.tx_error_count, (unsigned long)diagnostics.rx_reply_count,
           (unsigned long)diagnostics.rx_ack_count,
           (unsigned long)diagnostics.rx_invalid_frame_count,
           (unsigned long)diagnostics.rx_resync_drop_count,
           (unsigned long)diagnostics.rx_unknown_motor_count,
           (unsigned long)diagnostics.query_timeout_count);
  }

  printf("[TEST][DRIVE] ID1 enable\r\n");
  drive_emm_En_Control(1, true, false);
  HAL_Delay(200);
  printf("[TEST][DRIVE] ID1 normal velocity CW\r\n");
  drive_emm_Vel_Control(1, ZDT_DIR_CW, 150, 10, false);
  for (poll_count = 0; poll_count < 100; ++poll_count)
  {
    HAL_Delay(20);
  }
  HAL_Delay(1000);
  drive_emm_Stop_Now(1, false);
  HAL_Delay(1000);
  printf("[TEST][DRIVE] ID1 normal velocity CCW\r\n");
  drive_emm_Vel_Control(1, ZDT_DIR_CCW, 150, 10, false);
  for (poll_count = 0; poll_count < 100; ++poll_count)
  {
    HAL_Delay(20);
  }
  HAL_Delay(1000);
  drive_emm_Stop_Now(1, false);
  HAL_Delay(1000);

  printf("[TEST][DRIVE] ID1 normal position CW\r\n");
  reached = 0U;
  if (drive_emm_GetMotorFeedback(1, &feedback) == HAL_OK && feedback.valid != 0U)
  {
    target_pulse = feedback.position + 3200;
    drive_emm_Pos_Control(1, ZDT_DIR_CW, 150, 10, 3200, false, false);
    for (poll_count = 0; poll_count < 250; ++poll_count)
    {
      if (drive_emm_IsMotorReached(1, target_pulse, 100, 500) != 0U)
      {
        reached = 1U;
        break;
      }
      HAL_Delay(20);
    }
    printf("[TEST][DRIVE] ID1 position CW reached=%u target=%ld\r\n",
           (unsigned int)reached, (long)target_pulse);
  }
  else
  {
    printf("[TEST][DRIVE] ID1 position CW feedback invalid\r\n");
  }
  drive_emm_Stop_Now(1, false);
  HAL_Delay(1000);

  printf("[TEST][DRIVE] ID1 normal position CCW\r\n");
  reached = 0U;
  if (drive_emm_GetMotorFeedback(1, &feedback) == HAL_OK && feedback.valid != 0U)
  {
    target_pulse = feedback.position - 3200;
    drive_emm_Pos_Control(1, ZDT_DIR_CCW, 150, 10, 3200, false, false);
    for (poll_count = 0; poll_count < 250; ++poll_count)
    {
      if (drive_emm_IsMotorReached(1, target_pulse, 100, 500) != 0U)
      {
        reached = 1U;
        break;
      }
      HAL_Delay(20);
    }
    printf("[TEST][DRIVE] ID1 position CCW reached=%u target=%ld\r\n",
           (unsigned int)reached, (long)target_pulse);
  }
  else
  {
    printf("[TEST][DRIVE] ID1 position CCW feedback invalid\r\n");
  }
  drive_emm_Stop_Now(1, false);
  HAL_Delay(1000);
  if (drive_emm_GetMotorFeedback(1, &feedback) == HAL_OK)
  {
    printf("[TEST][DRIVE] ID1 feedback pos=%ld speed=%d en=%u stall=%u fault=%u valid=%u\r\n",
           (long)feedback.position, (int)feedback.speed_rpm,
           (unsigned int)feedback.enabled, (unsigned int)feedback.stalled,
           (unsigned int)feedback.fault, (unsigned int)feedback.valid);
  }
  drive_emm_En_Control(1, false, false);
  printf("[TEST][DRIVE] ID1 complete\r\n");

  printf("[TEST][DRIVE] ID2 enable\r\n");
  drive_emm_En_Control(2, true, false);
  HAL_Delay(200);
  printf("[TEST][DRIVE] ID2 normal velocity CW\r\n");
  drive_emm_Vel_Control(2, ZDT_DIR_CW, 150, 10, false);
  for (poll_count = 0; poll_count < 100; ++poll_count)
  {
    HAL_Delay(20);
  }
  drive_emm_Stop_Now(2, false);
  HAL_Delay(1000);
  printf("[TEST][DRIVE] ID2 normal velocity CCW\r\n");
  drive_emm_Vel_Control(2, ZDT_DIR_CCW, 150, 10, false);
  for (poll_count = 0; poll_count < 100; ++poll_count)
  {
    HAL_Delay(20);
  }
  drive_emm_Stop_Now(2, false);
  HAL_Delay(1000);
  printf("[TEST][DRIVE] ID2 normal position CW\r\n");
  reached = 0U;
  if (drive_emm_GetMotorFeedback(2, &feedback) == HAL_OK && feedback.valid != 0U)
  {
    target_pulse = feedback.position + 3200;
    drive_emm_Pos_Control(2, ZDT_DIR_CW, 150, 10, 3200, false, false);
    for (poll_count = 0; poll_count < 250; ++poll_count)
    {
      if (drive_emm_IsMotorReached(2, target_pulse, 100, 500) != 0U)
      {
        reached = 1U;
        break;
      }
      HAL_Delay(20);
    }
    printf("[TEST][DRIVE] ID2 position CW reached=%u target=%ld\r\n", (unsigned int)reached, (long)target_pulse);
  }
  else
  {
    printf("[TEST][DRIVE] ID2 position CW feedback invalid\r\n");
  }
  drive_emm_Stop_Now(2, false);
  HAL_Delay(1000);
  printf("[TEST][DRIVE] ID2 normal position CCW\r\n");
  reached = 0U;
  if (drive_emm_GetMotorFeedback(2, &feedback) == HAL_OK && feedback.valid != 0U)
  {
    target_pulse = feedback.position - 3200;
    drive_emm_Pos_Control(2, ZDT_DIR_CCW, 150, 10, 3200, false, false);
    for (poll_count = 0; poll_count < 250; ++poll_count)
    {
      if (drive_emm_IsMotorReached(2, target_pulse, 100, 500) != 0U)
      {
        reached = 1U;
        break;
      }
      HAL_Delay(20);
    }
    printf("[TEST][DRIVE] ID2 position CCW reached=%u target=%ld\r\n", (unsigned int)reached, (long)target_pulse);
  }
  else
  {
    printf("[TEST][DRIVE] ID2 position CCW feedback invalid\r\n");
  }
  drive_emm_Stop_Now(2, false);
  HAL_Delay(1000);
  if (drive_emm_GetMotorFeedback(2, &feedback) == HAL_OK)
  {
    printf("[TEST][DRIVE] ID2 feedback pos=%ld speed=%d en=%u stall=%u fault=%u valid=%u\r\n",
           (long)feedback.position, (int)feedback.speed_rpm,
           (unsigned int)feedback.enabled, (unsigned int)feedback.stalled,
           (unsigned int)feedback.fault, (unsigned int)feedback.valid);
  }
  drive_emm_En_Control(2, false, false);
  printf("[TEST][DRIVE] ID2 complete\r\n");

  printf("[TEST][DRIVE] ID3 enable\r\n");
  drive_emm_En_Control(3, true, false);
  HAL_Delay(200);
  printf("[TEST][DRIVE] ID3 normal velocity CW\r\n");
  drive_emm_Vel_Control(3, ZDT_DIR_CW, 150, 10, false);
  for (poll_count = 0; poll_count < 100; ++poll_count)
  {
    HAL_Delay(20);
  }
  drive_emm_Stop_Now(3, false);
  HAL_Delay(1000);
  printf("[TEST][DRIVE] ID3 normal velocity CCW\r\n");
  drive_emm_Vel_Control(3, ZDT_DIR_CCW, 150, 10, false);
  for (poll_count = 0; poll_count < 100; ++poll_count)
  {
    HAL_Delay(20);
  }
  drive_emm_Stop_Now(3, false);
  HAL_Delay(1000);
  printf("[TEST][DRIVE] ID3 normal position CW\r\n");
  reached = 0U;
  if (drive_emm_GetMotorFeedback(3, &feedback) == HAL_OK && feedback.valid != 0U)
  {
    target_pulse = feedback.position + 3200;
    drive_emm_Pos_Control(3, ZDT_DIR_CW, 150, 10, 3200, false, false);
    for (poll_count = 0; poll_count < 250; ++poll_count)
    {
      if (drive_emm_IsMotorReached(3, target_pulse, 100, 500) != 0U)
      {
        reached = 1U;
        break;
      }
      HAL_Delay(20);
    }
    printf("[TEST][DRIVE] ID3 position CW reached=%u target=%ld\r\n", (unsigned int)reached, (long)target_pulse);
  }
  else
  {
    printf("[TEST][DRIVE] ID3 position CW feedback invalid\r\n");
  }
  drive_emm_Stop_Now(3, false);
  HAL_Delay(1000);
  printf("[TEST][DRIVE] ID3 normal position CCW\r\n");
  reached = 0U;
  if (drive_emm_GetMotorFeedback(3, &feedback) == HAL_OK && feedback.valid != 0U)
  {
    target_pulse = feedback.position - 3200;
    drive_emm_Pos_Control(3, ZDT_DIR_CCW, 150, 10, 3200, false, false);
    for (poll_count = 0; poll_count < 250; ++poll_count)
    {
      if (drive_emm_IsMotorReached(3, target_pulse, 100, 500) != 0U)
      {
        reached = 1U;
        break;
      }
      HAL_Delay(20);
    }
    printf("[TEST][DRIVE] ID3 position CCW reached=%u target=%ld\r\n", (unsigned int)reached, (long)target_pulse);
  }
  else
  {
    printf("[TEST][DRIVE] ID3 position CCW feedback invalid\r\n");
  }
  drive_emm_Stop_Now(3, false);
  HAL_Delay(1000);
  if (drive_emm_GetMotorFeedback(3, &feedback) == HAL_OK)
  {
    printf("[TEST][DRIVE] ID3 feedback pos=%ld speed=%d en=%u stall=%u fault=%u valid=%u\r\n",
           (long)feedback.position, (int)feedback.speed_rpm,
           (unsigned int)feedback.enabled, (unsigned int)feedback.stalled,
           (unsigned int)feedback.fault, (unsigned int)feedback.valid);
  }
  drive_emm_En_Control(3, false, false);
  printf("[TEST][DRIVE] ID3 complete\r\n");

  printf("[TEST][DRIVE] ID4 enable\r\n");
  drive_emm_En_Control(4, true, false);
  HAL_Delay(200);
  printf("[TEST][DRIVE] ID4 normal velocity CW\r\n");
  drive_emm_Vel_Control(4, ZDT_DIR_CW, 150, 10, false);
  for (poll_count = 0; poll_count < 100; ++poll_count)
  {
    HAL_Delay(20);
  }
  drive_emm_Stop_Now(4, false);
  HAL_Delay(1000);
  printf("[TEST][DRIVE] ID4 normal velocity CCW\r\n");
  drive_emm_Vel_Control(4, ZDT_DIR_CCW, 150, 10, false);
  for (poll_count = 0; poll_count < 100; ++poll_count)
  {
    HAL_Delay(20);
  }
  drive_emm_Stop_Now(4, false);
  HAL_Delay(1000);
  printf("[TEST][DRIVE] ID4 normal position CW\r\n");
  reached = 0U;
  if (drive_emm_GetMotorFeedback(4, &feedback) == HAL_OK && feedback.valid != 0U)
  {
    target_pulse = feedback.position + 3200;
    drive_emm_Pos_Control(4, ZDT_DIR_CW, 150, 10, 3200, false, false);
    for (poll_count = 0; poll_count < 250; ++poll_count)
    {
      if (drive_emm_IsMotorReached(4, target_pulse, 100, 500) != 0U)
      {
        reached = 1U;
        break;
      }
      HAL_Delay(20);
    }
    printf("[TEST][DRIVE] ID4 position CW reached=%u target=%ld\r\n", (unsigned int)reached, (long)target_pulse);
  }
  else
  {
    printf("[TEST][DRIVE] ID4 position CW feedback invalid\r\n");
  }
  drive_emm_Stop_Now(4, false);
  HAL_Delay(1000);
  printf("[TEST][DRIVE] ID4 normal position CCW\r\n");
  reached = 0U;
  if (drive_emm_GetMotorFeedback(4, &feedback) == HAL_OK && feedback.valid != 0U)
  {
    target_pulse = feedback.position - 3200;
    drive_emm_Pos_Control(4, ZDT_DIR_CCW, 150, 10, 3200, false, false);
    for (poll_count = 0; poll_count < 250; ++poll_count)
    {
      if (drive_emm_IsMotorReached(4, target_pulse, 100, 500) != 0U)
      {
        reached = 1U;
        break;
      }
      HAL_Delay(20);
    }
    printf("[TEST][DRIVE] ID4 position CCW reached=%u target=%ld\r\n", (unsigned int)reached, (long)target_pulse);
  }
  else
  {
    printf("[TEST][DRIVE] ID4 position CCW feedback invalid\r\n");
  }
  drive_emm_Stop_Now(4, false);
  HAL_Delay(1000);
  if (drive_emm_GetMotorFeedback(4, &feedback) == HAL_OK)
  {
    printf("[TEST][DRIVE] ID4 feedback pos=%ld speed=%d en=%u stall=%u fault=%u valid=%u\r\n",
           (long)feedback.position, (int)feedback.speed_rpm,
           (unsigned int)feedback.enabled, (unsigned int)feedback.stalled,
           (unsigned int)feedback.fault, (unsigned int)feedback.valid);
  }
  drive_emm_En_Control(4, false, false);
  drive_emm_Stop_Now(1, false);
  drive_emm_Stop_Now(2, false);
  drive_emm_Stop_Now(3, false);
  drive_emm_Stop_Now(4, false);
  if (drive_emm_GetDiagnostics(&diagnostics) == HAL_OK)
  {
    printf("[TEST][DRIVE] diag q=%u/%u active=%u wait=%u monitor=%u txerr=%lu rx=%lu ack=%lu bad=%lu drop=%lu unknown=%lu timeout=%lu\r\n",
           (unsigned int)diagnostics.tx_queue_count, (unsigned int)diagnostics.tx_queue_depth,
           (unsigned int)diagnostics.tx_active, (unsigned int)diagnostics.query_waiting,
           (unsigned int)diagnostics.feedback_monitor_enabled,
           (unsigned long)diagnostics.tx_error_count, (unsigned long)diagnostics.rx_reply_count,
           (unsigned long)diagnostics.rx_ack_count,
           (unsigned long)diagnostics.rx_invalid_frame_count,
           (unsigned long)diagnostics.rx_resync_drop_count,
           (unsigned long)diagnostics.rx_unknown_motor_count,
           (unsigned long)diagnostics.query_timeout_count);
  }
  printf("[TEST] drive normal control modes complete\r\n");
}

void AdvanceTest_ScrewMotor(uint8_t id, bool is_x)
{
  const uint16_t vel_rpm = 150U;
  const uint16_t vel_deg_0p1 = vel_rpm * 10U;
  const uint8_t acc = 10U;
  const uint32_t one_turn_deg_0p1 = 3600U;

  drive_emm_En_Control(id, true, false);
  HAL_Delay(200U);

  if (is_x)
  {
    drive_emm_SetSpeedX(id, ZDT_DIR_CCW, vel_deg_0p1, acc, false);
    HAL_Delay(2000U);
    drive_emm_Stop_Now(id, false);
    HAL_Delay(1000U);
    drive_emm_SetSpeedX(id, ZDT_DIR_CW, vel_deg_0p1, acc, false);
    HAL_Delay(2000U);
    drive_emm_Stop_Now(id, false);
    HAL_Delay(1000U);
    drive_emm_SetTrapezoidPositionX(id, ZDT_DIR_CCW, 100U, 100U,
                                    vel_deg_0p1, one_turn_deg_0p1,
                                    ZDT_POS_RELATIVE_CURRENT, false);
    HAL_Delay(3000U);
    return;
  }

  drive_emm_Vel_Control(id, 0U, vel_rpm, acc, false);
  HAL_Delay(2000U);
  drive_emm_Stop_Now(id, false);
  HAL_Delay(1000U);
  drive_emm_Vel_Control(id, 1U, vel_rpm, acc, false);
  HAL_Delay(2000U);
  drive_emm_Stop_Now(id, false);
  HAL_Delay(1000U);
}

/* 依赖顺序：
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

*/


// ----------------------------------------------------------------------------------
// CHASIS TEST
// chasis 需要设置相关车长、轮子相关数据，请进行标定

/**
 * @brief 测试底盘电机正反转方向是否与底盘约定一致。
 */
void Test_Chasis_sign(void)
{
  printf("[TEST] chasis sign test\r\n");
  Chassis_Enable(true);
  HAL_Delay(1000);

  Chassis_SetMotorRPMEx(200, 200, 200, 200, 50);
  HAL_Delay(1000);
  Chassis_Stop();
  HAL_Delay(1000);
  Chassis_SetMotorRPMEx(-200, -200, -200, -200, 50);
  HAL_Delay(1000);
  Chassis_SmoothStop(50);
  HAL_Delay(1000);
}

/**
 * @brief  Chassis_SetBodyVelocityEx(float vx_right_mm_s, float vy_forward_mm_s, float wz_ccw_deg_s, uint8_t acc);
 */
void Test_Chasis_SetBodyVelocityEx(void)
{
  printf("[TEST] chasis SetBodyVelocityEx test\r\n");
  Chassis_Enable(true);
  HAL_Delay(1000);

  // 预期：车体向右移动，其他方向不动
  Chassis_SetBodyVelocityEx(200.0f, 0.0f, 0.0f, 50);
  HAL_Delay(1000);
  Chassis_Stop();
  HAL_Delay(1000);

  // 预期：车体向前移动，其他方向不动
  Chassis_SetBodyVelocityEx(0.0f, 200.0f, 0.0f, 50);
  HAL_Delay(1000);
  Chassis_Stop();
  HAL_Delay(1000);

  // 预期：车体逆时针旋转，其他方向不动
  Chassis_SetBodyVelocityEx(0.0f, 0.0f, 90.0f, 50);
  HAL_Delay(1000);
  Chassis_Stop();
  HAL_Delay(1000);

  // 预期：车体顺时针旋转，其他方向不动
  Chassis_SetBodyVelocityEx(0.0f, 0.0f, -90.0f, 50);
  HAL_Delay(1000);
  Chassis_Stop();
  HAL_Delay(1000);

  // 预期：车体向右移动并逆时针旋转，其他方向不动
  Chassis_SetBodyVelocityEx(200.0f, 0.0f, 90.0f, 50);
  HAL_Delay(1000);
  Chassis_Stop();
  HAL_Delay(1000);
}

/**
 * @brief  Chassis_MoveMecanumEx(int16_t forward_rpm, int16_t strafe_rpm, int16_t wz_ccw_rpm, uint8_t acc);
 */
void Test_Chasis_MoveMecanumEx(void)
{ 
  printf("[TEST] chasis MoveMecanumEx test\r\n");
  Chassis_Enable(true);
  HAL_Delay(1000);

  // 预期：车体向右移动并逆时针旋转，其他方向不动
  Chassis_MoveMecanumEx(0.0, 200.0f, 90.0f, 50);
  HAL_Delay(1000);
  Chassis_Stop();
  HAL_Delay(1000);

  // 预期：车体向前并向左运动，其他方向不动
  Chassis_MoveMecanumEx(200.0f, -200.0f, 0.0f, 50);
  HAL_Delay(1000);
  Chassis_Stop();
  HAL_Delay(1000);
}

/**
 * @brief 测试底盘使能、平滑停车、立即停车及运动命令状态。
 */
void Test_Chasis_EnableAndStop(void)
{
  printf("[TEST] chasis EnableAndStop test\r\n");
  Chassis_Enable(true);
  HAL_Delay(1000);

  // 预期：底盘向前运动，运动命令状态为1
  Chassis_SetMotorRPMEx(150, 150, 150, 150, 50);
  HAL_Delay(1000);
  printf("[TEST] chasis motion active=%u\r\n", (unsigned int)Chassis_IsMotionCommandActive());

  // 预期：底盘平滑停止，运动命令状态为0
  Chassis_SmoothStop(50);
  HAL_Delay(1000);
  printf("[TEST] chasis smooth stop active=%u\r\n", (unsigned int)Chassis_IsMotionCommandActive());

  // 预期：底盘立即停止，运动命令状态为0
  Chassis_Stop();
  HAL_Delay(1000);
  printf("[TEST] chasis stop active=%u\r\n", (unsigned int)Chassis_IsMotionCommandActive());
  Chassis_Enable(false);
}

/**
 * @brief 测试 Chassis_SetMotorRPMEx 的差速轮速与限速特殊用例。
 */
void Test_Chasis_SetMotorRPMExSpecial(void)
{
  printf("[TEST] chasis SetMotorRPMEx special test\r\n");
  Chassis_Enable(true);
  HAL_Delay(1000);

  // 预期：四个车轮按各自设置的正反转速度运行
  Chassis_SetMotorRPMEx(200, -150, 100, -50, 50);
  HAL_Delay(1000);
  Chassis_Stop();
  HAL_Delay(1000);

  // 预期：超过 CHASSIS_MAX_RPM 的速度被限制在最大转速内
  Chassis_SetMotorRPMEx(3200, -3200, 3200, -3200, 50);
  HAL_Delay(1000);
  Chassis_Stop();
  HAL_Delay(1000);
  Chassis_Enable(false);
}

/**
 * @brief 测试 Chassis_SetBodyVelocityEx 的反向组合与零速度特殊用例。
 */
void Test_Chasis_SetBodyVelocityExSpecial(void)
{
  printf("[TEST] chasis SetBodyVelocityEx special test\r\n");
  Chassis_Enable(true);
  HAL_Delay(1000);

  // 预期：车体向左后方移动并顺时针旋转
  Chassis_SetBodyVelocityEx(-150.0f, -150.0f, -60.0f, 50);
  HAL_Delay(1000);
  Chassis_Stop();
  HAL_Delay(1000);

  // 预期：零速度命令使底盘保持停止，运动命令状态为0
  Chassis_SetBodyVelocityEx(0.0f, 0.0f, 0.0f, 50);
  HAL_Delay(1000);
  printf("[TEST] chasis zero velocity active=%u\r\n", (unsigned int)Chassis_IsMotionCommandActive());
  Chassis_Enable(false);
}

/**
 * @brief 测试 Chassis_MoveMecanumEx 的原地旋转与三自由度组合用例。
 */
void Test_Chasis_MoveMecanumExSpecial(void)
{
  printf("[TEST] chasis MoveMecanumEx special test\r\n");
  Chassis_Enable(true);
  HAL_Delay(1000);

  // 预期：车体逆时针原地旋转，前后和横移方向不动
  Chassis_MoveMecanumEx(0, 0, 150, 50);
  HAL_Delay(1000);
  Chassis_Stop();
  HAL_Delay(1000);

  // 预期：车体同时前进、向右横移并逆时针旋转
  Chassis_MoveMecanumEx(150, 100, 50, 50);
  HAL_Delay(1000);
  Chassis_Stop();
  HAL_Delay(1000);
  Chassis_Enable(false);
}

// ----------------------------------------------------------------------------------
