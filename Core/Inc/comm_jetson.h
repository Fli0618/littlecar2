#ifndef COMM_JETSON_H
#define COMM_JETSON_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define DETECT_DEFAULT_PERIOD_MS ((uint16_t)40U)
#define DETECT_TARGET_MAX ((uint8_t)8U)

typedef enum
{
  DETECT_STATUS_OK = 0,
  DETECT_STATUS_NO_TARGET,
  DETECT_STATUS_BAD_COMMAND,
  DETECT_STATUS_BAD_LENGTH,
  DETECT_STATUS_BAD_PERIOD,
  DETECT_STATUS_UART_ERROR
} Detect_Status_t;

typedef struct
{
  uint8_t type;
  int16_t x;
  int16_t y;
  uint8_t confidence;
  uint8_t measured;
  uint8_t support_count;
} Detect_Target_t;

typedef struct
{
  uint8_t count;
  Detect_Target_t targets[DETECT_TARGET_MAX];
} Detect_TargetList_t;

typedef struct
{
  Detect_Status_t status;
  int16_t x;
  int16_t y;
  uint8_t support_count;
  uint8_t measured_count;
} Detect_DiskCenter_t;

Detect_Status_t detect_color_start(void);
Detect_Status_t detect_circle_start(void);
Detect_Status_t detect_disk_center_start(void);
Detect_Status_t detect_stop(void);

uint8_t detect_get_targets(Detect_TargetList_t *result);
uint8_t detect_get_disk_center(Detect_DiskCenter_t *result);
uint8_t detect_is_active(void);
uint8_t detect_is_fresh(uint32_t timeout_ms);

void CommJetson_Init(UART_HandleTypeDef *huart);
void CommJetson_OnUartRxEvent(UART_HandleTypeDef *huart, uint16_t size);
void CommJetson_OnUartError(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
