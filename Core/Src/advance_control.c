#include "advance_control.h"

/* 唯一的底盘控制权状态；周期控制器和顺序业务都只通过本模块访问。 */
static volatile AdvanceControl_Mode_t g_control_mode = ADVANCE_CONTROL_NONE;

/*
 * 统一取消依赖的控制器取消接口。仅作外部声明，避免控制权模块反向包含
 * AdvanceMotion / AdvanceHolonomic 的重型头文件。
 */
void AdvanceMotion_Cancel(void);
void AdvanceHolonomic_Cancel(void);

void AdvanceControl_Init(void)
{
  /* 初始化不停车，避免启动阶段向尚未完成初始化的电机队列写命令。 */
  g_control_mode = ADVANCE_CONTROL_NONE;
}

uint8_t AdvanceControl_SetMode(AdvanceControl_Mode_t mode)
{
  if ((mode != ADVANCE_CONTROL_NONE) &&
      (mode != ADVANCE_CONTROL_WORLD) &&
      (mode != ADVANCE_CONTROL_VISUAL) &&
      (mode != ADVANCE_CONTROL_HOLONOMIC))
  {
    return 0U;
  }

  /* 活动控制器必须先释放到 NONE，防止两个控制源同时向底盘发命令。 */
  if ((mode != ADVANCE_CONTROL_NONE) &&
      (g_control_mode != ADVANCE_CONTROL_NONE) &&
      (g_control_mode != mode))
  {
    return 0U;
  }

  if ((mode == ADVANCE_CONTROL_NONE) &&
      (g_control_mode != ADVANCE_CONTROL_NONE))
  {
    /* Chassis_Stop() 只入队 DMA 帧，不等待发送完成，适合在 TIM6 中调用。 */
    Chassis_Stop();
  }

  g_control_mode = mode;
  return 1U;
}

uint8_t AdvanceControl_ReleaseMode(void)
{
  g_control_mode = ADVANCE_CONTROL_NONE;
  return 1U;
}

AdvanceControl_Mode_t AdvanceControl_GetMode(void)
{
  return g_control_mode;
}

uint8_t AdvanceControl_IsBusy(void)
{
  return (g_control_mode != ADVANCE_CONTROL_NONE) ? 1U : 0U;
}

void AdvanceControl_CancelActive(void)
{
  switch (g_control_mode)
  {
  case ADVANCE_CONTROL_WORLD:
    AdvanceMotion_Cancel();
    break;

  case ADVANCE_CONTROL_HOLONOMIC:
    AdvanceHolonomic_Cancel();
    break;

  default:
    /* NONE / VISUAL：无远程控制器可取消，不发送停车 */
    break;
  }
}
