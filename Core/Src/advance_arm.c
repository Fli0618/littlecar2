#include "advance_arm.h"
#include "drive_bus_servo.h"
#include "drive_emm.h"
#include "sensor_limit.h"

void AdvanceArm_Init(void)
{
}

void AdvanceArm_LiftHomeBlocking(void)
{
  uint32_t started_tick;

  if (SensorLimit_IsLiftHomeActive())
  {
    drive_emm_Vel_Control(ARM_LIFT_MOTOR_ID, ARM_LIFT_DOWN_DIRECTION, ARM_HOME_RELEASE_SPEED, ARM_HOME_ACC, false);
    started_tick = HAL_GetTick();

    while (SensorLimit_IsLiftHomeActive())
    {
      if ((HAL_GetTick() - started_tick) > ARM_HOME_RELEASE_TIMEOUT_MS)
      {
        drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
        return;
      }
      HAL_Delay(1U);
    }

    drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
    HAL_Delay(100U);
  }

  drive_emm_Vel_Control(ARM_LIFT_MOTOR_ID, ARM_LIFT_UP_DIRECTION, ARM_HOME_SPEED, ARM_HOME_ACC, false);
  started_tick = HAL_GetTick();

  while (!SensorLimit_IsLiftHomeActive())
  {
    if ((HAL_GetTick() - started_tick) > ARM_HOME_TIMEOUT_MS)
    {
      drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
      return;
    }
    HAL_Delay(1U);
  }

  drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
  HAL_Delay(100U);
  drive_emm_Reset_CurPos_To_Zero(ARM_LIFT_MOTOR_ID);
  HAL_Delay(100U);
}

void AdvanceArm_SlideSetCurrentAsZero(void)
{
  drive_emm_Stop_Now(ARM_SLIDE_MOTOR_ID, false);
  HAL_Delay(100U);
  drive_emm_Reset_CurPos_To_Zero(ARM_SLIDE_MOTOR_ID);
  HAL_Delay(100U);
}

void AdvanceArm_MoveLiftToBlocking(uint32_t position_pulse)
{
  if (position_pulse > ARM_LIFT_POS_MAX)
  {
    return;
  }

  drive_emm_Pos_Control(ARM_LIFT_MOTOR_ID, ARM_LIFT_ABSOLUTE_DIRECTION, ARM_LIFT_SPEED, ARM_LIFT_ACC, position_pulse, true, false);
  HAL_Delay(5000U);
}

void AdvanceArm_MoveSlideToBlocking(uint32_t position_pulse)
{
  if (position_pulse > ARM_SLIDE_POS_MAX)
  {
    return;
  }

  drive_emm_Pos_Control(ARM_SLIDE_MOTOR_ID, ARM_SLIDE_ABSOLUTE_DIRECTION, ARM_SLIDE_SPEED, ARM_SLIDE_ACC, position_pulse, true, false);
  HAL_Delay(5000U);
}

void AdvanceArm_GripperOpen(void)
{
  (void)BusServo_SetPositionEx(ARM_GRIPPER_SERVO_ID, ARM_GRIPPER_ACC, ARM_GRIPPER_OPEN_POS, ARM_GRIPPER_SPEED);
  HAL_Delay(1000U);
}

void AdvanceArm_GripperClose(void)
{
  (void)BusServo_SetPositionEx(ARM_GRIPPER_SERVO_ID, ARM_GRIPPER_ACC, ARM_GRIPPER_CLOSE_POS, ARM_GRIPPER_SPEED);
  HAL_Delay(1000U);
}

void AdvanceArm_RotateOutwardCenter(void)
{
  (void)BusServo_SetPositionEx(ARM_ROTATE_SERVO_ID, ARM_ROTATE_ACC, ARM_ROTATE_POS_OUTWARD_CENTER, ARM_ROTATE_SPEED);
  HAL_Delay(1000U);
}

void AdvanceArm_RotateOutwardLeft(void)
{
  (void)BusServo_SetPositionEx(ARM_ROTATE_SERVO_ID, ARM_ROTATE_ACC, ARM_ROTATE_POS_OUTWARD_LEFT, ARM_ROTATE_SPEED);
  HAL_Delay(1000U);
}

void AdvanceArm_RotateOutwardRight(void)
{
  (void)BusServo_SetPositionEx(ARM_ROTATE_SERVO_ID, ARM_ROTATE_ACC, ARM_ROTATE_POS_OUTWARD_RIGHT, ARM_ROTATE_SPEED);
  HAL_Delay(1000U);
}

void AdvanceArm_RotateToTray(void)
{
  (void)BusServo_SetPositionEx(ARM_ROTATE_SERVO_ID, ARM_ROTATE_ACC, ARM_ROTATE_POS_TRAY, ARM_ROTATE_SPEED);
  HAL_Delay(1000U);
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
  (void)BusServo_SetPositionEx(ARM_MATERIAL_SERVO_ID, ARM_MATERIAL_ACC, ARM_MATERIAL_POS_1, ARM_MATERIAL_SPEED);
  HAL_Delay(1000U);
}

void AdvanceArm_TraySlot2(void)
{
  (void)BusServo_SetPositionEx(ARM_MATERIAL_SERVO_ID, ARM_MATERIAL_ACC, ARM_MATERIAL_POS_2, ARM_MATERIAL_SPEED);
  HAL_Delay(1000U);
}

void AdvanceArm_TraySlot3(void)
{
  (void)BusServo_SetPositionEx(ARM_MATERIAL_SERVO_ID, ARM_MATERIAL_ACC, ARM_MATERIAL_POS_3, ARM_MATERIAL_SPEED);
  HAL_Delay(1000U);
}

void AdvanceArm_Stop(void)
{
  drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
  drive_emm_Stop_Now(ARM_SLIDE_MOTOR_ID, false);
}

void AdvanceArm_EStop(void)
{
  AdvanceArm_Stop();
}
