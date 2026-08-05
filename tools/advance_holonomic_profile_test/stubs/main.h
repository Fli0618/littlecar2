#ifndef STUB_MAIN_H
#define STUB_MAIN_H

/*
 * 主机侧测试阴影头：屏蔽真实 STM32 main.h，
 * 仅提供被测模块需要的 HAL_GetTick 声明。
 */

#include <stdint.h>

uint32_t HAL_GetTick(void);

#endif
