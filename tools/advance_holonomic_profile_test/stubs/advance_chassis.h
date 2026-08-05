#ifndef STUB_ADVANCE_CHASSIS_H
#define STUB_ADVANCE_CHASSIS_H

/*
 * 主机侧测试阴影头：仅声明被测模块用到的 Chassis 接口，
 * 避免引入真实 drive_emm.h（其依赖 STM32 HAL 类型）。
 */

#include <stdbool.h>
#include <stdint.h>

uint8_t Chassis_SetBodyVelocityEx(float vx_right_mm_s, float vy_forward_mm_s,
                                  float wz_ccw_deg_s, uint8_t acc);
uint8_t Chassis_Stop(void);
uint8_t Chassis_SmoothStop(uint8_t acc);

#endif
