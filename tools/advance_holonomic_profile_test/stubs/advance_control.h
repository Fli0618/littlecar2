#ifndef STUB_ADVANCE_CONTROL_H
#define STUB_ADVANCE_CONTROL_H

/*
 * 主机侧测试阴影头：控制权枚举与函数声明，数值与真实模块一致。
 */

#include "advance_chassis.h"

typedef enum
{
  ADVANCE_CONTROL_NONE = 0U,
  ADVANCE_CONTROL_WORLD,
  ADVANCE_CONTROL_VISUAL,
  ADVANCE_CONTROL_HOLONOMIC
} AdvanceControl_Mode_t;

uint8_t AdvanceControl_SetMode(AdvanceControl_Mode_t mode);
uint8_t AdvanceControl_ReleaseMode(void);
AdvanceControl_Mode_t AdvanceControl_GetMode(void);
void AdvanceControl_CancelActive(void);

#endif
