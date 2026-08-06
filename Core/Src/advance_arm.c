#include "advance_arm.h"
#include "drive_bus_servo.h"
#include "drive_emm.h"
#include "sensor_limit.h"

/* 升降轴只有在光电归零成功后才允许执行绝对位置运动。 */
static bool g_lift_homed = false;

static void AdvanceArm_DelayBlocking(uint32_t delay_ms)
{
  uint32_t started_tick = HAL_GetTick();

  while ((HAL_GetTick() - started_tick) < delay_ms)
  {
    __WFI();
  }
}

static bool AdvanceArm_IsFeedbackValid(uint8_t motor_id,
                                       DriveEmm_MotorFeedback_t *feedback)
{
  if ((drive_emm_GetMotorFeedback(motor_id, feedback) != HAL_OK) ||
      (feedback->valid == 0U) ||
      ((HAL_GetTick() - feedback->updated_tick) >
       DRIVE_EMM_ARM_FEEDBACK_TIMEOUT_MS) ||
      (feedback->stalled != 0U) ||
      (feedback->fault != 0U))
  {
    return false;
  }

  return true;
}

static void AdvanceArm_MoveServoBlocking(uint8_t servo_id,
                                         uint16_t acceleration,
                                         int32_t position,
                                         uint16_t speed)
{
  (void)BusServo_SetPositionEx(servo_id, acceleration, position, speed);
  AdvanceArm_DelayBlocking(ARM_SERVO_MOVE_DELAY_MS);
}

void AdvanceArm_Init(void)
{
  g_lift_homed = false;

  (void)drive_emm_MonitorMotor(ARM_LIFT_MOTOR_ID);
  (void)drive_emm_MonitorMotor(ARM_SLIDE_MOTOR_ID);
}

void AdvanceArm_LiftHomeBlocking(void)
{
  uint32_t started_tick;
  uint32_t confirm_tick;

  g_lift_homed = false;

  if (SensorLimit_IsActive(SENSOR_LIMIT_LIFT_UP))
  {
    if (SensorLimit_IsActive(SENSOR_LIMIT_LIFT_DOWN))
    {
      drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
      return;
    }

    drive_emm_Vel_Control(ARM_LIFT_MOTOR_ID,
                          ARM_LIFT_DOWN_DIRECTION,
                          ARM_HOME_RELEASE_SPEED,
                          ARM_HOME_ACC,
                          false);
    started_tick = HAL_GetTick();

    while (SensorLimit_IsActive(SENSOR_LIMIT_LIFT_UP))
    {
      if (SensorLimit_IsActive(SENSOR_LIMIT_LIFT_DOWN) ||
          ((HAL_GetTick() - started_tick) > ARM_HOME_RELEASE_TIMEOUT_MS))
      {
        drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
        return;
      }
      __WFI();
    }

    drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
    AdvanceArm_DelayBlocking(ARM_HOME_COMMAND_DELAY_MS);
  }

  drive_emm_Vel_Control(ARM_LIFT_MOTOR_ID,
                        ARM_LIFT_UP_DIRECTION,
                        ARM_HOME_SPEED,
                        ARM_HOME_ACC,
                        false);
  started_tick = HAL_GetTick();

  while (1)
  {
    if (SensorLimit_IsActive(SENSOR_LIMIT_LIFT_UP))
    {
      confirm_tick = HAL_GetTick();
      while ((HAL_GetTick() - confirm_tick) < ARM_HOME_CONFIRM_MS)
      {
        if (!SensorLimit_IsActive(SENSOR_LIMIT_LIFT_UP))
        {
          break;
        }
        if ((HAL_GetTick() - started_tick) > ARM_HOME_TIMEOUT_MS)
        {
          drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
          return;
        }
        __WFI();
      }

      if (SensorLimit_IsActive(SENSOR_LIMIT_LIFT_UP))
      {
        break;
      }
    }

    if ((HAL_GetTick() - started_tick) > ARM_HOME_TIMEOUT_MS)
    {
      drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
      return;
    }
    __WFI();
  }

  drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
  AdvanceArm_DelayBlocking(ARM_HOME_COMMAND_DELAY_MS);
  drive_emm_Reset_CurPos_To_Zero(ARM_LIFT_MOTOR_ID);
  AdvanceArm_DelayBlocking(ARM_HOME_COMMAND_DELAY_MS);
  g_lift_homed = true;
}

void AdvanceArm_SlideSetCurrentAsZero(void)
{
  drive_emm_Stop_Now(ARM_SLIDE_MOTOR_ID, false);
  AdvanceArm_DelayBlocking(ARM_HOME_COMMAND_DELAY_MS);
  drive_emm_Reset_CurPos_To_Zero(ARM_SLIDE_MOTOR_ID);
  AdvanceArm_DelayBlocking(ARM_HOME_COMMAND_DELAY_MS);
}

void AdvanceArm_MoveLiftToBlocking(uint32_t position_pulse)
{
  DriveEmm_MotorFeedback_t feedback;
  SensorLimitId_t movement_limit;
  uint32_t started_tick;
  int32_t error;

  if (!g_lift_homed || (position_pulse > ARM_LIFT_POS_MAX))
  {
    return;
  }

  if (!AdvanceArm_IsFeedbackValid(ARM_LIFT_MOTOR_ID, &feedback))
  {
    drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
    g_lift_homed = false;
    return;
  }

  error = feedback.position - (int32_t)position_pulse;
  if (error < 0)
  {
    error = -error;
  }
  if (error <= ARM_POSITION_TOLERANCE_PULSE)
  {
    return;
  }

  movement_limit = ((int32_t)position_pulse > feedback.position)
                       ? SENSOR_LIMIT_LIFT_DOWN
                       : SENSOR_LIMIT_LIFT_UP;
  if (SensorLimit_IsActive(movement_limit))
  {
    drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
    return;
  }

  drive_emm_Pos_Control(ARM_LIFT_MOTOR_ID,
                        ARM_LIFT_ABSOLUTE_DIRECTION,
                        ARM_LIFT_SPEED,
                        ARM_LIFT_ACC,
                        position_pulse,
                        true,
                        false);
  started_tick = HAL_GetTick();

  while (1)
  {
    if (SensorLimit_IsActive(movement_limit))
    {
      drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
      return;
    }

    if (!AdvanceArm_IsFeedbackValid(ARM_LIFT_MOTOR_ID, &feedback))
    {
      drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
      g_lift_homed = false;
      return;
    }

    if (drive_emm_IsMotorReached(ARM_LIFT_MOTOR_ID,
                                 (int32_t)position_pulse,
                                 ARM_POSITION_TOLERANCE_PULSE,
                                 DRIVE_EMM_ARM_FEEDBACK_TIMEOUT_MS) != 0U)
    {
      return;
    }

    if ((HAL_GetTick() - started_tick) > ARM_MOVE_TIMEOUT_MS)
    {
      drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
      g_lift_homed = false;
      return;
    }
    __WFI();
  }
}

void AdvanceArm_MoveSlideToBlocking(uint32_t position_pulse)
{
  DriveEmm_MotorFeedback_t feedback;
  uint32_t started_tick;
  int32_t error;

  if (position_pulse > ARM_SLIDE_POS_MAX)
  {
    return;
  }

  if (!AdvanceArm_IsFeedbackValid(ARM_SLIDE_MOTOR_ID, &feedback))
  {
    drive_emm_Stop_Now(ARM_SLIDE_MOTOR_ID, false);
    return;
  }

  error = feedback.position - (int32_t)position_pulse;
  if (error < 0)
  {
    error = -error;
  }
  if (error <= ARM_POSITION_TOLERANCE_PULSE)
  {
    return;
  }

  drive_emm_Pos_Control(ARM_SLIDE_MOTOR_ID,
                        ARM_SLIDE_ABSOLUTE_DIRECTION,
                        ARM_SLIDE_SPEED,
                        ARM_SLIDE_ACC,
                        position_pulse,
                        true,
                        false);
  started_tick = HAL_GetTick();

  while (1)
  {
    if (!AdvanceArm_IsFeedbackValid(ARM_SLIDE_MOTOR_ID, &feedback))
    {
      drive_emm_Stop_Now(ARM_SLIDE_MOTOR_ID, false);
      return;
    }

    if (drive_emm_IsMotorReached(ARM_SLIDE_MOTOR_ID,
                                 (int32_t)position_pulse,
                                 ARM_POSITION_TOLERANCE_PULSE,
                                 DRIVE_EMM_ARM_FEEDBACK_TIMEOUT_MS) != 0U)
    {
      return;
    }

    if ((HAL_GetTick() - started_tick) > ARM_MOVE_TIMEOUT_MS)
    {
      drive_emm_Stop_Now(ARM_SLIDE_MOTOR_ID, false);
      return;
    }
    __WFI();
  }
}

void AdvanceArm_Grab(bool closed)
{
  AdvanceArm_MoveServoBlocking(ARM_GRIPPER_SERVO_ID,
                               ARM_GRIPPER_ACC,
                               closed ? ARM_GRIPPER_CLOSE_POS
                                      : ARM_GRIPPER_OPEN_POS,
                               ARM_GRIPPER_SPEED);
}

void AdvanceArm_GripperOpen(void)
{
  AdvanceArm_Grab(false);
}

void AdvanceArm_GripperClose(void)
{
  AdvanceArm_Grab(true);
}

void AdvanceArm_RotateToPickup(void)
{
  AdvanceArm_MoveServoBlocking(ARM_ROTATE_SERVO_ID,
                               ARM_ROTATE_ACC,
                               ARM_ROTATE_POS_PICKUP,
                               ARM_ROTATE_SPEED);
}

void AdvanceArm_RotateToTray(void)
{
  AdvanceArm_MoveServoBlocking(ARM_ROTATE_SERVO_ID,
                               ARM_ROTATE_ACC,
                               ARM_ROTATE_POS_TRAY,
                               ARM_ROTATE_SPEED);
}

void AdvanceArm_SlideToPickupBlocking(void)
{
  AdvanceArm_MoveSlideToBlocking(ARM_SLIDE_POS_PICKUP);
}

void AdvanceArm_SlideToTrayBlocking(void)
{
  AdvanceArm_MoveSlideToBlocking(ARM_SLIDE_POS_TRAY);
}

void AdvanceArm_LiftLowBlocking(void)
{
  AdvanceArm_MoveLiftToBlocking(ARM_LIFT_POS_LOW);
}

void AdvanceArm_LiftHighBlocking(void)
{
  AdvanceArm_MoveLiftToBlocking(ARM_LIFT_POS_HIGH);
}

void AdvanceArm_LiftToPickupBlocking(void)
{
  AdvanceArm_MoveLiftToBlocking(ARM_LIFT_POS_PICKUP);
}

void AdvanceArm_LiftToTrayBlocking(void)
{
  AdvanceArm_MoveLiftToBlocking(ARM_LIFT_POS_TRAY);
}

void AdvanceArm_LiftToStackBlocking(void)
{
  AdvanceArm_MoveLiftToBlocking(ARM_LIFT_POS_STACK);
}

void AdvanceArm_TraySlot1(void)
{
  AdvanceArm_MoveServoBlocking(ARM_MATERIAL_SERVO_ID,
                               ARM_MATERIAL_ACC,
                               ARM_MATERIAL_POS_1,
                               ARM_MATERIAL_SPEED);
}

void AdvanceArm_TraySlot2(void)
{
  AdvanceArm_MoveServoBlocking(ARM_MATERIAL_SERVO_ID,
                               ARM_MATERIAL_ACC,
                               ARM_MATERIAL_POS_2,
                               ARM_MATERIAL_SPEED);
}

void AdvanceArm_TraySlot3(void)
{
  AdvanceArm_MoveServoBlocking(ARM_MATERIAL_SERVO_ID,
                               ARM_MATERIAL_ACC,
                               ARM_MATERIAL_POS_3,
                               ARM_MATERIAL_SPEED);
}

void AdvanceArm_Stop(void)
{
  drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
  drive_emm_Stop_Now(ARM_SLIDE_MOTOR_ID, false);
}

void AdvanceArm_EStop(void)
{
  AdvanceArm_Stop();
  g_lift_homed = false;
}
