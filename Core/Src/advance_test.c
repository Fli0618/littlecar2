#include "advance_test.h"

#include <stdio.h>

#include "drive_emm.h"
#include "main.h"
#include "advance_holonomic_position.h"

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

void Test_emm_1(void)
{
  drive_emm_Pos_Control(1, 1, 200, 60, 1000, 1, 0);
}
void Test_emm_2(void)
{
  
}
void Test_emm_3(void)
{
  
}

/**
 * @brief 对底盘多电机同步前进、后退流程进行实机测试。
 * @note 测试前必须将车辆架空，并确认现场具备断电或急停条件。
 */
void Test_MMCL(void)
{
  const int16_t test_rpm = 150;
  const uint8_t test_acc = 10U;
  DriveEmm_Diagnostics_t diagnostics;

  printf("[TEST][MMCL] start: wheels must be off the ground\r\n");
  printf("[TEST][MMCL] enable motors\r\n");
  Chassis_Enable(true);
  HAL_Delay(200U);

  printf("[TEST][MMCL] forward: rpm=%d acc=%u duration=2000ms\r\n",
         (int)test_rpm, (unsigned int)test_acc);
  Chassis_SetMotorRPMEx(test_rpm, test_rpm, test_rpm, test_rpm, test_acc);
  HAL_Delay(2000U);

  printf("[TEST][MMCL] stop after forward\r\n");
  Chassis_Stop();
  HAL_Delay(500U);

  printf("[TEST][MMCL] reverse: rpm=%d acc=%u duration=2000ms\r\n",
         -(int)test_rpm, (unsigned int)test_acc);
  Chassis_SetMotorRPMEx(-test_rpm, -test_rpm, -test_rpm, -test_rpm, test_acc);
  HAL_Delay(2000U);

  printf("[TEST][MMCL] stop after reverse\r\n");
  Chassis_Stop();
  HAL_Delay(500U);

  printf("[TEST][MMCL] disable motors\r\n");
  Chassis_Enable(false);

  if (drive_emm_GetDiagnostics(&diagnostics) == HAL_OK)
  {
    printf("[TEST][MMCL] diag txerr=%lu rx=%lu ack=%lu bad=%lu timeout=%lu\r\n",
           (unsigned long)diagnostics.tx_error_count,
           (unsigned long)diagnostics.rx_reply_count,
           (unsigned long)diagnostics.rx_ack_count,
           (unsigned long)diagnostics.rx_invalid_frame_count,
           (unsigned long)diagnostics.query_timeout_count);
  }
  printf("[TEST][MMCL] complete\r\n");
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

// SERVO TEST
void Test_Servo1(void)
{
  printf("[TEST] test servo\r\n");
  BusServo_SetPositionEx(3, 0, 4096, 0);
  HAL_Delay(2000);
  BusServo_SetPositionEx(3, 0, 0, 0);
  HAL_Delay(1000);
}


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
  Chassis_SetBodyVelocityEx(150.0f, 0.0f, 0.0f, 50);
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
  Chassis_MoveMecanumEx(0.0, 200.0f, 180.0f, 50);
  HAL_Delay(1000);
  Chassis_Stop();
  HAL_Delay(1000);

  // 预期：车体向前并向左运动，其他方向不动
  Chassis_MoveMecanumEx(200.0f, -200.0f, 0.0f, 50);
  HAL_Delay(1000);
  Chassis_Stop();
  HAL_Delay(1000);
}


void AdvanceTest_PrintImuOpsData(void)
{
  const volatile WIT_Data_t *wit_data;
  OPS_Pose_t ops_pose = {0};
  OPS_Status_t ops_status;

  wit_data = WIT_GetData();
  ops_status = OPS_GetPose(&ops_pose);

  printf("[CAL][WIT] accel_g x=%.3f y=%.3f z=%.3f valid=%u tick=%lu\r\n",
         (double)wit_data->accel_g.x, (double)wit_data->accel_g.y,
         (double)wit_data->accel_g.z, (unsigned int)wit_data->accel_g.valid,
         (unsigned long)wit_data->accel_g.updated_tick);
  printf("[CAL][WIT] gyro_dps x=%.3f y=%.3f z=%.3f valid=%u tick=%lu\r\n",
         (double)wit_data->gyro_dps.x, (double)wit_data->gyro_dps.y,
         (double)wit_data->gyro_dps.z, (unsigned int)wit_data->gyro_dps.valid,
         (unsigned long)wit_data->gyro_dps.updated_tick);
  printf("[CAL][WIT] angle_deg x=%.3f y=%.3f z=%.3f valid=%u tick=%lu\r\n",
         (double)wit_data->angle_deg.x, (double)wit_data->angle_deg.y,
         (double)wit_data->angle_deg.z, (unsigned int)wit_data->angle_deg.valid,
         (unsigned long)wit_data->angle_deg.updated_tick);

  printf("[CAL][OPS] status=%d valid=%u frame=%lu tick=%lu\r\n",
         (int)ops_status, (unsigned int)ops_pose.valid,
         (unsigned long)ops_pose.frame_count, (unsigned long)ops_pose.updated_tick);
  printf("[CAL][OPS] angle_deg z=%.3f x=%.3f y=%.3f wz_dps=%.3f\r\n",
         (double)ops_pose.zangle_deg, (double)ops_pose.xangle_deg,
         (double)ops_pose.yangle_deg, (double)ops_pose.w_z_dps);
  printf("[CAL][OPS] pos_mm x=%.3f y=%.3f\r\n",
         (double)ops_pose.pos_x_mm, (double)ops_pose.pos_y_mm);
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
  status = AdvanceMotion_SetWorldVelocityEx(800.0f, 0.0f, 0.0f, 80);
  printf("[TEST] motion world velocity x status=%d\r\n", (int)status);
  HAL_Delay(2000);
  // Chassis_Stop();
  Chassis_SmoothStop(120);
  HAL_Delay(1000);

  // 预期：车体沿世界坐标系 Y 轴负方向运动并顺时针旋转
  status = AdvanceMotion_SetWorldVelocityEx(0.0f, -800.0f, 0, 80);
  printf("[TEST] motion world velocity y status=%d\r\n", (int)status);
  HAL_Delay(2000);
  // Chassis_Stop();
  Chassis_SmoothStop(120);
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
  state = AdvanceMotion_GotoPoseBlocking(pose.x_mm + 300.0f, pose.y_mm + 300,
                                          pose.yaw_deg, 80);
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

/**
 * @brief 测试 AdvanceHolonomic_GotoPoseBlocking 的到点运动。
 * @note 保守目标：相对当前位置前进 300 mm 并保持当前航向；
 *       实车五步调试时在此临时修改目标与增益。
 */
void Test_Holonomic_GotoPoseBlocking(void)
{
  AdvanceWorld_Status_t world_status;
  AdvanceHolonomic_RunState_t state;
  WorldGoalPose2D_t goal;
  WorldPose2D_t pose;

  printf("[TEST] holonomic GotoPoseBlocking test\r\n");
  AdvanceHolonomic_Init();
  Chassis_Enable(true);
  HAL_Delay(1000);

  world_status = AdvanceWorld_GetPoseCopy(&pose);
  if (world_status != ADVANCE_WORLD_STATUS_OK)
  {
    printf("[TEST] holonomic pose unavailable status=%d\r\n", (int)world_status);
    Chassis_Enable(false);
    return;
  }

  // 预期：车体沿当前航向前进 300 mm，到达后自动停车并释放控制权
  goal.x_mm = pose.x_mm;
  goal.y_mm = pose.y_mm + 300.0f;
  goal.yaw_deg = pose.yaw_deg;
  goal.vmax_mm_s = 300.0f;
  goal.wmax_deg_s = 60.0f;
  goal.timeout_ms = 8000U;
  goal.goal_flags = ADVANCE_HOLONOMIC_GOAL_USE_POSITION | ADVANCE_HOLONOMIC_GOAL_USE_YAW;
  state = AdvanceHolonomic_GotoGoalBlocking(&goal, CHASSIS_DEFAULT_ACC);
  printf("[TEST] holonomic result state=%d\r\n", (int)state);
  Chassis_Stop();
  Chassis_Enable(false);
}

static void AdvanceTest_PrintYawFreshnessSnapshot(const char *source_name)
{
  AdvanceMotion_DebugSnapshot_t snapshot;
  AdvanceMotion_Status_t status;
  uint8_t yaw_valid;
  uint32_t yaw_updated_tick;

  status = AdvanceMotion_GetDebugSnapshot(&snapshot);
  if (status != ADVANCE_MOTION_STATUS_OK)
  {
    printf("[TEST][YAW] %s snapshot status=%d\r\n", source_name, (int)status);
    return;
  }

  if (AdvanceWorld_GetYawSource() == ADVANCE_WORLD_YAW_SOURCE_OPS)
  {
    yaw_valid = snapshot.pose.ops_yaw_valid;
    yaw_updated_tick = snapshot.pose.ops_yaw_updated_tick;
  }
  else
  {
    yaw_valid = snapshot.pose.wit_yaw_valid;
    yaw_updated_tick = snapshot.pose.wit_yaw_updated_tick;
  }

  printf("[TEST][YAW] source=%s flags=0x%02X pose_fresh=%u yaw_fresh=%u valid=%u yaw_valid=%u yaw_tick=%lu\r\n",
         source_name, (unsigned int)snapshot.flags,
         (unsigned int)((snapshot.flags & ADVANCE_MOTION_DEBUG_FLAG_POSE_FRESH) != 0U),
         (unsigned int)((snapshot.flags & ADVANCE_MOTION_DEBUG_FLAG_YAW_FRESH) != 0U),
         (unsigned int)((snapshot.flags & ADVANCE_MOTION_DEBUG_FLAG_VALID) != 0U),
         (unsigned int)yaw_valid, (unsigned long)yaw_updated_tick);
}

void AdvanceTest_VerifyYawSourceFreshness(void)
{
  AdvanceMotion_RuntimeStatus_t motion_status;
  AdvanceMotion_Status_t status;
  AdvanceWorld_YawSource_t original_source;
  AdvanceWorld_YawSource_t sources[2] = {
      ADVANCE_WORLD_YAW_SOURCE_WIT,
      ADVANCE_WORLD_YAW_SOURCE_OPS};
  uint8_t index;

  status = AdvanceMotion_GetStatus(&motion_status);
  if ((status != ADVANCE_MOTION_STATUS_OK) ||
      ((motion_status.state != ADVANCE_MOTION_STATE_IDLE) &&
       (motion_status.state != ADVANCE_MOTION_STATE_ARRIVED) &&
       (motion_status.state != ADVANCE_MOTION_STATE_CANCELED) &&
       (motion_status.state != ADVANCE_MOTION_STATE_TIMEOUT) &&
       (motion_status.state != ADVANCE_MOTION_STATE_NO_POSE) &&
       (motion_status.state != ADVANCE_MOTION_STATE_NO_ORIGIN) &&
       (motion_status.state != ADVANCE_MOTION_STATE_OFF_PATH)))
  {
    printf("[TEST][YAW] motion active, skip source switch\r\n");
    return;
  }

  original_source = AdvanceWorld_GetYawSource();
  printf("[TEST][YAW] freshness switch test start\r\n");
  for (index = 0U; index < 2U; ++index)
  {
    if (AdvanceWorld_SetYawSource(sources[index]) != ADVANCE_WORLD_STATUS_OK)
    {
      printf("[TEST][YAW] source=%u switch failed\r\n", (unsigned int)sources[index]);
      continue;
    }
    AdvanceMotion_ResetYawControl();
    HAL_Delay(25U);
    AdvanceTest_PrintYawFreshnessSnapshot((sources[index] == ADVANCE_WORLD_YAW_SOURCE_OPS) ? "OPS" : "WIT");
  }

  if (AdvanceWorld_SetYawSource(original_source) == ADVANCE_WORLD_STATUS_OK)
  {
    AdvanceMotion_ResetYawControl();
    printf("[TEST][YAW] source restored=%u\r\n", (unsigned int)original_source);
  }
  else
  {
    printf("[TEST][YAW] source restore failed=%u\r\n", (unsigned int)original_source);
  }
}

// ----------------------------------------------------------------------------------

void Test_jetson(Competition_StartArea_t start_area)
{
  printf("[TEST][JETSON] start\r\n");
  Detect_TargetList_t target_list = {0};
  Detect_Status_t status;
  uint8_t received;
  uint32_t started_tick;
  uint32_t target_frame_count = 0U;
  uint32_t qr_frame_count = 0U;
  const uint32_t observe_ms = 20000U;
  const uint32_t poll_period_ms = 20U;
  char code[DETECT_QR_CODE_LENGTH + 1U] = {0};

  printf("[TEST][JETSON] competition start area=%u (%s)\r\n",
          (unsigned int)start_area,
          (start_area == COMPETITION_START_AREA_1) ? "AREA_1" : "AREA_2");

  status = detect_color_start();
  printf("[TEST][JETSON] color detection start status=%u\r\n", (unsigned int)status);
  if (status == DETECT_STATUS_OK)
  {
    started_tick = HAL_GetTick();
    while ((HAL_GetTick() - started_tick) < observe_ms)
    {
      received = detect_get_targets(&target_list);
      if (received != 0U)
      {
        ++target_frame_count;
        printf("[TEST][JETSON] target frame=%lu count=%u\r\n",
               (unsigned long)target_frame_count, (unsigned int)target_list.count);
        for (uint8_t i = 0U; i < target_list.count; ++i)
        {
          const Detect_Target_t *target = &target_list.targets[i];
          printf("[TEST][JETSON] target[%u] type=%u x=%d y=%d confidence=%u measured=%u support=%u\r\n",
                 (unsigned int)i, (unsigned int)target->type, (int)target->x, (int)target->y,
                 (unsigned int)target->confidence, (unsigned int)target->measured,
                 (unsigned int)target->support_count);
        }
      }
      HAL_Delay(poll_period_ms);
    }
    printf("[TEST][JETSON] color detection finished frames=%lu\r\n",
           (unsigned long)target_frame_count);
    if (target_frame_count == 0U)
    {
      printf("[TEST][JETSON] target result: no result in %lu ms\r\n",
             (unsigned long)observe_ms);
    }
    status = detect_stop();
    printf("[TEST][JETSON] color detection stop status=%u\r\n", (unsigned int)status);
  }
  else
  {
    printf("[TEST][JETSON] color detection skipped due to start failure\r\n");
  }

  status = detect_qr_start();
  printf("[TEST][JETSON] QR detection start status=%u\r\n", (unsigned int)status);
  if (status == DETECT_STATUS_OK)
  {
    started_tick = HAL_GetTick();
    while ((HAL_GetTick() - started_tick) < observe_ms)
    {
      if (detect_get_qr(code) != 0U)
      {
        ++qr_frame_count;
        printf("[TEST][JETSON] QR frame=%lu code=%s\r\n",
               (unsigned long)qr_frame_count, code);
      }
      HAL_Delay(poll_period_ms);
    }
    printf("[TEST][JETSON] QR detection finished frames=%lu\r\n",
           (unsigned long)qr_frame_count);
    if (qr_frame_count == 0U)
    {
      printf("[TEST][JETSON] QR result: no result in %lu ms\r\n",
             (unsigned long)observe_ms);
    }
    status = detect_stop();
    printf("[TEST][JETSON] QR detection stop status=%u\r\n", (unsigned int)status);
  }
  else
  {
    printf("[TEST][JETSON] QR detection skipped due to start failure\r\n");
  }
}
