#include "advance_arm.h"
#include "drive_bus_servo.h"
#include "drive_emm.h"
#include "sensor_limit.h"

static void AdvanceArm_MoveServoBlocking(uint8_t servo_id,
                                         uint16_t acceleration,
                                         int32_t position,
                                         uint16_t speed)
{
  (void)BusServo_SetPositionEx(servo_id, acceleration, position, speed);
  HAL_Delay(ARM_SERVO_MOVE_DELAY_MS);
}

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
      HAL_Delay(ARM_HOME_POLL_DELAY_MS);
    }

    drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
    HAL_Delay(ARM_HOME_COMMAND_DELAY_MS);
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
    HAL_Delay(ARM_HOME_POLL_DELAY_MS);
  }

  drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
  HAL_Delay(ARM_HOME_COMMAND_DELAY_MS);
  drive_emm_Reset_CurPos_To_Zero(ARM_LIFT_MOTOR_ID);
  HAL_Delay(ARM_HOME_COMMAND_DELAY_MS);
}

void AdvanceArm_SlideSetCurrentAsZero(void)
{
  drive_emm_Stop_Now(ARM_SLIDE_MOTOR_ID, false);
  HAL_Delay(ARM_HOME_COMMAND_DELAY_MS);
  drive_emm_Reset_CurPos_To_Zero(ARM_SLIDE_MOTOR_ID);
  HAL_Delay(ARM_HOME_COMMAND_DELAY_MS);
}

void AdvanceArm_MoveLiftToBlocking(uint32_t position_pulse)
{
  if (position_pulse > ARM_LIFT_POS_MAX)
  {
    return;
  }

  drive_emm_Pos_Control(ARM_LIFT_MOTOR_ID, ARM_LIFT_ABSOLUTE_DIRECTION, ARM_LIFT_SPEED, ARM_LIFT_ACC, position_pulse, true, false);
  HAL_Delay(ARM_LIFT_MOVE_DELAY_MS);
}

void AdvanceArm_MoveSlideToBlocking(uint32_t position_pulse)
{
  if (position_pulse > ARM_SLIDE_POS_MAX)
  {
    return;
  }

  drive_emm_Pos_Control(ARM_SLIDE_MOTOR_ID, ARM_SLIDE_ABSOLUTE_DIRECTION, ARM_SLIDE_SPEED, ARM_SLIDE_ACC, position_pulse, true, false);
  HAL_Delay(ARM_SLIDE_MOVE_DELAY_MS);
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

void AdvanceArm_RotateOutwardCenter(void)
{
  AdvanceArm_MoveServoBlocking(ARM_ROTATE_SERVO_ID,
                               ARM_ROTATE_ACC,
                               ARM_ROTATE_POS_OUTWARD_CENTER,
                               ARM_ROTATE_SPEED);
}

void AdvanceArm_RotateOutwardLeft(void)
{
  AdvanceArm_MoveServoBlocking(ARM_ROTATE_SERVO_ID,
                               ARM_ROTATE_ACC,
                               ARM_ROTATE_POS_OUTWARD_LEFT,
                               ARM_ROTATE_SPEED);
}

void AdvanceArm_RotateOutwardRight(void)
{
  AdvanceArm_MoveServoBlocking(ARM_ROTATE_SERVO_ID,
                               ARM_ROTATE_ACC,
                               ARM_ROTATE_POS_OUTWARD_RIGHT,
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
}
