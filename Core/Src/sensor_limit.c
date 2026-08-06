#include "sensor_limit.h"

GPIO_PinState SensorLimit_ReadLiftHomeLevel(void)
{
  return HAL_GPIO_ReadPin(LIFT_UP_LIMIT_GPIO_Port, LIFT_UP_LIMIT_Pin);
}

bool SensorLimit_IsLiftHomeActive(void)
{
  return SensorLimit_ReadLiftHomeLevel() == SENSOR_LIMIT_ACTIVE_LEVEL;
}
