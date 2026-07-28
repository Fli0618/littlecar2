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
// CHASSIS TEST
// chassis 需要设置相关车长、轮子相关数据，请进行标定

/**
 * @brief 测试底盘电机正反转方向是否与底盘约定一致。
 */
void Test_Chassis_Sign(void)
{
  printf("[TEST] chassis sign test\r\n");
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
void Test_Chassis_SetBodyVelocityEx(void)
{
  printf("[TEST] chassis SetBodyVelocityEx test\r\n");
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
void Test_Chassis_MoveMecanumEx(void)
{ 
  printf("[TEST] chassis MoveMecanumEx test\r\n");
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

// ----------------------------------------------------------------------------------
// MOTION TEST
// motion 依赖世界坐标数据，请确认 OPS、WIT 已正常更新且世界原点已建立

/**
 * @brief 测试 AdvanceMotion_SetWorldVelocityEx 的世界坐标系速度控制。
 */
void Test_Motion_SetWorldVelocityEx(void)
{
  AdvanceMotion_Status_t status;

  printf("[TEST] motion SetWorldVelocityEx test\r\n");
  AdvanceMotion_Init();
  Chassis_Enable(true);
  HAL_Delay(1000);

  // 预期：车体沿世界坐标系 X 轴正方向运动，航向角保持不变
  status = AdvanceMotion_SetWorldVelocityEx(150.0f, 0.0f, 0.0f, 50);
  printf("[TEST] motion world velocity x status=%d\r\n", (int)status);
  HAL_Delay(1000);
  Chassis_Stop();
  HAL_Delay(1000);

  // 预期：车体沿世界坐标系 Y 轴负方向运动并顺时针旋转
  status = AdvanceMotion_SetWorldVelocityEx(0.0f, -150.0f, -45.0f, 50);
  printf("[TEST] motion world velocity y status=%d\r\n", (int)status);
  HAL_Delay(1000);
  Chassis_Stop();
  Chassis_Enable(false);
}

/**
 * @brief 测试 AdvanceMotion_GotoPoseBlocking 的位置到点动作。
 */
void Test_Motion_GotoPoseBlocking(void)
{
  AdvanceWorld_Status_t world_status;
  AdvanceMotion_RunState_t state;
  WorldPose2D_t pose;
  WorldGoalPose2D_t goal;

  printf("[TEST] motion GotoPoseBlocking test\r\n");
  AdvanceMotion_Init();
  Chassis_Enable(true);
  HAL_Delay(1000);

  world_status = AdvanceWorld_GetPoseCopy(&pose);
  if (world_status != ADVANCE_WORLD_STATUS_OK)
  {
    printf("[TEST] motion pose unavailable status=%d\r\n", (int)world_status);
    Chassis_Enable(false);
    return;
  }

  // 预期：车体保持当前航向，移动至世界坐标系 X 轴正方向 300mm 处
  goal.x_mm = pose.x_mm + 300.0f;
  goal.y_mm = pose.y_mm;
  goal.yaw_deg = pose.yaw_deg;
  goal.vmax_mm_s = 150.0f;
  goal.wmax_deg_s = 0.0f;
  goal.timeout_ms = 10000U;
  goal.goal_flags = 0U;
  state = AdvanceMotion_GotoGoalBlocking(&goal, 50);
  printf("[TEST] motion goto position state=%d\r\n", (int)state);
  Chassis_Stop();
  Chassis_Enable(false);
}

/**
 * @brief 测试 AdvanceMotion_GotoPoseEx 的航向到点与取消动作。
 */
void Test_Motion_GotoPoseYawAndCancel(void)
{
  AdvanceWorld_Status_t world_status;
  AdvanceMotion_RuntimeStatus_t motion_status;
  AdvanceMotion_Status_t status;
  WorldPose2D_t pose;
  WorldGoalPose2D_t goal;

  printf("[TEST] motion GotoPoseYawAndCancel test\r\n");
  AdvanceMotion_Init();
  Chassis_Enable(true);
  HAL_Delay(1000);

  world_status = AdvanceWorld_GetPoseCopy(&pose);
  if (world_status != ADVANCE_WORLD_STATUS_OK)
  {
    printf("[TEST] motion pose unavailable status=%d\r\n", (int)world_status);
    Chassis_Enable(false);
    return;
  }

  // 预期：车体保持当前位置，逆时针旋转 45 度后到点
  goal.x_mm = pose.x_mm;
  goal.y_mm = pose.y_mm;
  goal.yaw_deg = AdvanceWorld_WrapAngleDeg(pose.yaw_deg + 45.0f);
  goal.vmax_mm_s = 0.0f;
  goal.wmax_deg_s = 45.0f;
  goal.timeout_ms = 10000U;
  goal.goal_flags = ADVANCE_MOTION_GOAL_USE_YAW;
  status = AdvanceMotion_GotoPoseEx(&goal, 50);
  printf("[TEST] motion goto yaw status=%d\r\n", (int)status);
  HAL_Delay(500);

  // 预期：取消当前到点任务，底盘停止且状态变为 CANCELED
  AdvanceMotion_CancelIfActive();
  status = AdvanceMotion_GetStatus(&motion_status);
  printf("[TEST] motion cancel status=%d state=%d\r\n", (int)status, (int)motion_status.state);
  Chassis_Stop();
  Chassis_Enable(false);
}

// ----------------------------------------------------------------------------------
