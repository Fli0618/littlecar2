#include "advance_arm.h"
#include "drive_bus_servo.h"
#include "drive_emm.h"
#include "sensor_limit.h"

/* 两根轴独立维护归零有效性，任一轴异常不影响另一轴。 */
static bool g_lift_homed = false;
static bool g_slide_homed = false;

/* 升降轴以顶部限位为零点；归零过程必须先释放已压住的限位。 */
static bool AdvanceArm_HomeLiftBlocking(void)
{
  uint32_t started_tick;
  uint32_t confirm_tick;

  g_lift_homed = false;
  if (SensorLimit_IsActive(SENSOR_LIMIT_LIFT_UP))
  {
    /* 已处于零点时先向下脱离，避免再次寻零立即被判定成功。 */
    if (SensorLimit_IsActive(SENSOR_LIMIT_LIFT_DOWN))
    {
      /* 对侧限位已有效，禁止继续向下释放。 */
      drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
      return false;
    }
    drive_emm_Vel_Control(ARM_LIFT_MOTOR_ID, ARM_LIFT_DOWN_DIRECTION,
                          ARM_HOME_RELEASE_SPEED, ARM_HOME_ACC, false);
    started_tick = HAL_GetTick();
    while (SensorLimit_IsActive(SENSOR_LIMIT_LIFT_UP))
    {
      if (SensorLimit_IsActive(SENSOR_LIMIT_LIFT_DOWN))
      {
        /* 释放过程中触发对侧限位，立即停止以防机构碰撞。 */
        drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
        return false;
      }
      if ((HAL_GetTick() - started_tick) > ARM_HOME_RELEASE_TIMEOUT_MS)
      {
        drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
        return false;
      }
      __WFI();
    }
    drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
    started_tick = HAL_GetTick();
    while ((HAL_GetTick() - started_tick) < ARM_HOME_COMMAND_DELAY_MS)
    {
      __WFI();
    }
  }

  /* 低速向上搜索零点，并对光电信号做时间确认。 */
  drive_emm_Vel_Control(ARM_LIFT_MOTOR_ID, ARM_LIFT_UP_DIRECTION,
                        ARM_HOME_SPEED, ARM_HOME_ACC, false);
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
          return false;
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
      return false;
    }
    __WFI();
  }

  /* 停止命令发送完成后再清零，避免当前位置与机械零点不同步。 */
  drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
  started_tick = HAL_GetTick();
  while ((HAL_GetTick() - started_tick) < ARM_HOME_COMMAND_DELAY_MS)
  {
    __WFI();
  }
  drive_emm_Reset_CurPos_To_Zero(ARM_LIFT_MOTOR_ID);
  started_tick = HAL_GetTick();
  while ((HAL_GetTick() - started_tick) < ARM_HOME_COMMAND_DELAY_MS)
  {
    __WFI();
  }
  g_lift_homed = true;
  return true;
}

/* 滑台轴以后部限位为零点；流程与升降轴保持独立。 */
static bool AdvanceArm_HomeSlideBlocking(void)
{
  uint32_t started_tick;
  uint32_t confirm_tick;

  g_slide_homed = false;
  if (SensorLimit_IsActive(SENSOR_LIMIT_SLIDE_REAR))
  {
    /* 已压住后限位时先向前释放。 */
    if (SensorLimit_IsActive(SENSOR_LIMIT_SLIDE_FRONT))
    {
      /* 对侧限位已有效，禁止继续向前释放。 */
      drive_emm_Stop_Now(ARM_SLIDE_MOTOR_ID, false);
      return false;
    }
    drive_emm_Vel_Control(ARM_SLIDE_MOTOR_ID, ARM_SLIDE_EXTEND_DIRECTION,
                          ARM_HOME_RELEASE_SPEED, ARM_HOME_ACC, false);
    started_tick = HAL_GetTick();
    while (SensorLimit_IsActive(SENSOR_LIMIT_SLIDE_REAR))
    {
      if (SensorLimit_IsActive(SENSOR_LIMIT_SLIDE_FRONT))
      {
        /* 释放过程中触发对侧限位，立即停止以防机构碰撞。 */
        drive_emm_Stop_Now(ARM_SLIDE_MOTOR_ID, false);
        return false;
      }
      if ((HAL_GetTick() - started_tick) > ARM_HOME_RELEASE_TIMEOUT_MS)
      {
        drive_emm_Stop_Now(ARM_SLIDE_MOTOR_ID, false);
        return false;
      }
      __WFI();
    }
    drive_emm_Stop_Now(ARM_SLIDE_MOTOR_ID, false);
    started_tick = HAL_GetTick();
    while ((HAL_GetTick() - started_tick) < ARM_HOME_COMMAND_DELAY_MS)
    {
      __WFI();
    }
  }

  /* 低速向后搜索零点，并对光电信号做时间确认。 */
  drive_emm_Vel_Control(ARM_SLIDE_MOTOR_ID, ARM_SLIDE_RETRACT_DIRECTION,
                        ARM_HOME_SPEED, ARM_HOME_ACC, false);
  started_tick = HAL_GetTick();
  while (1)
  {
    if (SensorLimit_IsActive(SENSOR_LIMIT_SLIDE_REAR))
    {
      confirm_tick = HAL_GetTick();
      while ((HAL_GetTick() - confirm_tick) < ARM_HOME_CONFIRM_MS)
      {
        if (!SensorLimit_IsActive(SENSOR_LIMIT_SLIDE_REAR))
        {
          break;
        }
        if ((HAL_GetTick() - started_tick) > ARM_HOME_TIMEOUT_MS)
        {
          drive_emm_Stop_Now(ARM_SLIDE_MOTOR_ID, false);
          return false;
        }
        __WFI();
      }
      if (SensorLimit_IsActive(SENSOR_LIMIT_SLIDE_REAR))
      {
        break;
      }
    }
    if ((HAL_GetTick() - started_tick) > ARM_HOME_TIMEOUT_MS)
    {
      drive_emm_Stop_Now(ARM_SLIDE_MOTOR_ID, false);
      return false;
    }
    __WFI();
  }

  /* 停止命令发送完成后再清零。 */
  drive_emm_Stop_Now(ARM_SLIDE_MOTOR_ID, false);
  started_tick = HAL_GetTick();
  while ((HAL_GetTick() - started_tick) < ARM_HOME_COMMAND_DELAY_MS)
  {
    __WFI();
  }
  drive_emm_Reset_CurPos_To_Zero(ARM_SLIDE_MOTOR_ID);
  started_tick = HAL_GetTick();
  while ((HAL_GetTick() - started_tick) < ARM_HOME_COMMAND_DELAY_MS)
  {
    __WFI();
  }
  g_slide_homed = true;
  return true;
}

void AdvanceArm_Init(void)
{
  /* 初始化只注册反馈与复位软件状态，不驱动机械臂。 */
  g_lift_homed = false;
  g_slide_homed = false;

  (void)drive_emm_MonitorMotor(ARM_LIFT_MOTOR_ID);
  (void)drive_emm_MonitorMotor(ARM_SLIDE_MOTOR_ID);
}

bool AdvanceArm_HomeBlocking(void)
{
  /* 串行归零可避免两个轴同时运动造成机构干涉。 */
  if (!g_lift_homed && !AdvanceArm_HomeLiftBlocking())
  {
    AdvanceArm_Stop();
    return false;
  }
  if (!g_slide_homed && !AdvanceArm_HomeSlideBlocking())
  {
    AdvanceArm_Stop();
    return false;
  }
  return AdvanceArm_IsHomed();
}

bool AdvanceArm_IsLiftHomed(void)
{
  return g_lift_homed;
}

bool AdvanceArm_IsSlideHomed(void)
{
  return g_slide_homed;
}

bool AdvanceArm_IsHomed(void)
{
  return g_lift_homed && g_slide_homed;
}

AdvanceArm_MoveStatus_t AdvanceArm_MoveLiftToBlocking(uint32_t position_pulse)
{
  DriveEmm_MotorFeedback_t feedback;
  SensorLimitId_t limit;
  uint32_t started_tick;
  int32_t error;

  /* 绝对坐标只在已知零点和可用反馈的前提下执行。 */
  if (!g_lift_homed)
  {
    return ADVANCE_ARM_MOVE_NOT_HOMED;
  }
  if (position_pulse > ARM_LIFT_POS_MAX)
  {
    return ADVANCE_ARM_MOVE_OUT_OF_RANGE;
  }
  if ((drive_emm_GetMotorFeedback(ARM_LIFT_MOTOR_ID, &feedback) != HAL_OK) ||
      (feedback.valid == 0U) ||
      ((HAL_GetTick() - feedback.updated_tick) > DRIVE_EMM_ARM_FEEDBACK_TIMEOUT_MS))
  {
    drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
    g_lift_homed = false;
    return ADVANCE_ARM_MOVE_FEEDBACK_ERROR;
  }
  if ((feedback.stalled != 0U) || (feedback.fault != 0U))
  {
    drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
    g_lift_homed = false;
    return ADVANCE_ARM_MOVE_MOTOR_FAULT;
  }
  error = feedback.position - (int32_t)position_pulse;
  if (error < 0)
  {
    error = -error;
  }
  if (error <= ARM_POSITION_TOLERANCE_PULSE)
  {
    return ADVANCE_ARM_MOVE_OK;
  }

  /* 只监测实际运动方向的限位，允许离开已触发的另一端限位。 */
  limit = ((int32_t)position_pulse > feedback.position) ?
              SENSOR_LIMIT_LIFT_DOWN : SENSOR_LIMIT_LIFT_UP;
  if (SensorLimit_IsActive(limit))
  {
    return ADVANCE_ARM_MOVE_LIMIT_REACHED;
  }
  drive_emm_Pos_Control(ARM_LIFT_MOTOR_ID, ARM_LIFT_ABSOLUTE_DIRECTION,
                        ARM_LIFT_SPEED, ARM_LIFT_ACC, position_pulse, true, false);
  started_tick = HAL_GetTick();
  /* 等待期间持续检查限位、反馈状态、到位条件和总超时。 */
  while (1)
  {
    if (SensorLimit_IsActive(limit))
    {
      drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
      return ADVANCE_ARM_MOVE_LIMIT_REACHED;
    }
    if ((drive_emm_GetMotorFeedback(ARM_LIFT_MOTOR_ID, &feedback) != HAL_OK) ||
        (feedback.valid == 0U) ||
        ((HAL_GetTick() - feedback.updated_tick) > DRIVE_EMM_ARM_FEEDBACK_TIMEOUT_MS))
    {
      drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
      g_lift_homed = false;
      return ADVANCE_ARM_MOVE_FEEDBACK_ERROR;
    }
    if ((feedback.stalled != 0U) || (feedback.fault != 0U))
    {
      drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
      g_lift_homed = false;
      return ADVANCE_ARM_MOVE_MOTOR_FAULT;
    }
    if (drive_emm_IsMotorReached(ARM_LIFT_MOTOR_ID, (int32_t)position_pulse,
                                  ARM_POSITION_TOLERANCE_PULSE,
                                  DRIVE_EMM_ARM_FEEDBACK_TIMEOUT_MS) != 0U)
    {
      return ADVANCE_ARM_MOVE_OK;
    }
    if ((HAL_GetTick() - started_tick) > ARM_MOVE_TIMEOUT_MS)
    {
      drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
      g_lift_homed = false;
      return ADVANCE_ARM_MOVE_TIMEOUT;
    }
    __WFI();
  }
}

AdvanceArm_MoveStatus_t AdvanceArm_MoveSlideToBlocking(uint32_t position_pulse)
{
  DriveEmm_MotorFeedback_t feedback;
  SensorLimitId_t limit;
  uint32_t started_tick;
  int32_t error;

  /* 绝对坐标只在已知零点和可用反馈的前提下执行。 */
  if (!g_slide_homed)
  {
    return ADVANCE_ARM_MOVE_NOT_HOMED;
  }
  if (position_pulse > ARM_SLIDE_POS_MAX)
  {
    return ADVANCE_ARM_MOVE_OUT_OF_RANGE;
  }
  if ((drive_emm_GetMotorFeedback(ARM_SLIDE_MOTOR_ID, &feedback) != HAL_OK) ||
      (feedback.valid == 0U) ||
      ((HAL_GetTick() - feedback.updated_tick) > DRIVE_EMM_ARM_FEEDBACK_TIMEOUT_MS))
  {
    drive_emm_Stop_Now(ARM_SLIDE_MOTOR_ID, false);
    g_slide_homed = false;
    return ADVANCE_ARM_MOVE_FEEDBACK_ERROR;
  }
  if ((feedback.stalled != 0U) || (feedback.fault != 0U))
  {
    drive_emm_Stop_Now(ARM_SLIDE_MOTOR_ID, false);
    g_slide_homed = false;
    return ADVANCE_ARM_MOVE_MOTOR_FAULT;
  }
  error = feedback.position - (int32_t)position_pulse;
  if (error < 0)
  {
    error = -error;
  }
  if (error <= ARM_POSITION_TOLERANCE_PULSE)
  {
    return ADVANCE_ARM_MOVE_OK;
  }

  /* 只监测实际运动方向的限位，允许离开已触发的另一端限位。 */
  limit = ((int32_t)position_pulse > feedback.position) ?
              SENSOR_LIMIT_SLIDE_FRONT : SENSOR_LIMIT_SLIDE_REAR;
  if (SensorLimit_IsActive(limit))
  {
    return ADVANCE_ARM_MOVE_LIMIT_REACHED;
  }
  drive_emm_Pos_Control(ARM_SLIDE_MOTOR_ID, ARM_SLIDE_ABSOLUTE_DIRECTION,
                        ARM_SLIDE_SPEED, ARM_SLIDE_ACC, position_pulse, true, false);
  started_tick = HAL_GetTick();
  /* 等待期间持续检查限位、反馈状态、到位条件和总超时。 */
  while (1)
  {
    if (SensorLimit_IsActive(limit))
    {
      drive_emm_Stop_Now(ARM_SLIDE_MOTOR_ID, false);
      return ADVANCE_ARM_MOVE_LIMIT_REACHED;
    }
    if ((drive_emm_GetMotorFeedback(ARM_SLIDE_MOTOR_ID, &feedback) != HAL_OK) ||
        (feedback.valid == 0U) ||
        ((HAL_GetTick() - feedback.updated_tick) > DRIVE_EMM_ARM_FEEDBACK_TIMEOUT_MS))
    {
      drive_emm_Stop_Now(ARM_SLIDE_MOTOR_ID, false);
      g_slide_homed = false;
      return ADVANCE_ARM_MOVE_FEEDBACK_ERROR;
    }
    if ((feedback.stalled != 0U) || (feedback.fault != 0U))
    {
      drive_emm_Stop_Now(ARM_SLIDE_MOTOR_ID, false);
      g_slide_homed = false;
      return ADVANCE_ARM_MOVE_MOTOR_FAULT;
    }
    if (drive_emm_IsMotorReached(ARM_SLIDE_MOTOR_ID, (int32_t)position_pulse,
                                  ARM_POSITION_TOLERANCE_PULSE,
                                  DRIVE_EMM_ARM_FEEDBACK_TIMEOUT_MS) != 0U)
    {
      return ADVANCE_ARM_MOVE_OK;
    }
    if ((HAL_GetTick() - started_tick) > ARM_MOVE_TIMEOUT_MS)
    {
      drive_emm_Stop_Now(ARM_SLIDE_MOTOR_ID, false);
      g_slide_homed = false;
      return ADVANCE_ARM_MOVE_TIMEOUT;
    }
    __WFI();
  }
}

void AdvanceArm_Grab(bool closed)
{
  int32_t position = closed ? ARM_GRIPPER_CLOSE_POS : ARM_GRIPPER_OPEN_POS;

  (void)BusServo_SetPositionEx(ARM_GRIPPER_SERVO_ID, ARM_GRIPPER_ACC,
                                position, ARM_GRIPPER_SPEED);
  HAL_Delay(1000U);
}

void AdvanceArm_Stop(void)
{
  /* 受控停止不改变已建立的零点坐标。 */
  drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
  drive_emm_Stop_Now(ARM_SLIDE_MOTOR_ID, false);
}

void AdvanceArm_EStop(void)
{
  /* 急停后的机械状态不可确认，两个轴均需重新归零。 */
  AdvanceArm_Stop();
  g_lift_homed = false;
  g_slide_homed = false;
}

// 相关高级动作还没有实现
