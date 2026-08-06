#ifndef SENSOR_LIMIT_H
#define SENSOR_LIMIT_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>

#include "main.h"

/** @brief 限位传感器的有效电平，可按实际接线在编译期覆盖。 */
#ifndef SENSOR_LIMIT_ACTIVE_LEVEL
#define SENSOR_LIMIT_ACTIVE_LEVEL GPIO_PIN_SET
#endif

/**
 * @brief 读取升降轴顶部归零光电的原始 GPIO 电平。
 * @return GPIO_PIN_SET 或 GPIO_PIN_RESET。
 */
GPIO_PinState SensorLimit_ReadLiftHomeLevel(void);

/**
 * @brief 判断升降轴顶部归零光电是否处于有效状态。
 * @return true 表示已触发归零光电，false 表示未触发。
 */
bool SensorLimit_IsLiftHomeActive(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_LIMIT_H */
