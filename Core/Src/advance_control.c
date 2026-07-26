#include "advance_control.h"

#include "advance_chassis.h"

static volatile AdvanceControl_Mode_t g_control_mode = ADVANCE_CONTROL_NONE;

void AdvanceControl_Init(void)
{
  g_control_mode = ADVANCE_CONTROL_NONE;
}

void AdvanceControl_SetMode(AdvanceControl_Mode_t mode)
{
  if ((mode != ADVANCE_CONTROL_NONE) &&
      (mode != ADVANCE_CONTROL_WORLD) &&
      (mode != ADVANCE_CONTROL_VISUAL))
  {
    return;
  }

  if ((mode != ADVANCE_CONTROL_NONE) &&
      (g_control_mode != ADVANCE_CONTROL_NONE) &&
      (g_control_mode != mode))
  {
    return;
  }

  if ((mode == ADVANCE_CONTROL_NONE) &&
      (g_control_mode != ADVANCE_CONTROL_NONE))
  {
    Chassis_Stop();
  }

  g_control_mode = mode;
}

AdvanceControl_Mode_t AdvanceControl_GetMode(void)
{
  return g_control_mode;
}
