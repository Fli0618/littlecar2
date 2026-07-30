#ifndef COMM_STDIO_H
#define COMM_STDIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "stm32f4xx_hal.h"

/**
 * @brief Bind stdout to a UART and select whether characters are transmitted.
 *
 * Call after the UART has been initialized. When output is disabled, stdio
 * remains retargeted away from semihosting but generated characters are
 * discarded so a binary protocol can own the UART.
 */
void CommStdio_Init(UART_HandleTypeDef *huart, uint8_t output_enabled);

#ifdef __cplusplus
}
#endif

#endif
