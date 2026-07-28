#ifndef COMM_TUNER_H
#define COMM_TUNER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"

/** 初始化 USART1 调参通信的 DMA + IDLE 接收。 */
HAL_StatusTypeDef CommTuner_Init(UART_HandleTypeDef *huart);
/** 在前台解析协议、执行业务动作并启动 DMA 响应发送。 */
void CommTuner_Process(void);
/** 由 TIM6 每 1 ms 调用，推进远程控制心跳超时保护。 */
void CommTuner_Update(void);
/** 转发 USART1 DMA + IDLE 接收事件；回调中仅搬运字节。 */
void CommTuner_OnUartRxEvent(UART_HandleTypeDef *huart, uint16_t size);
/** 转发 USART1 DMA 发送完成事件。 */
void CommTuner_OnUartTxComplete(UART_HandleTypeDef *huart);
/** 转发 USART1 错误事件并恢复接收。 */
void CommTuner_OnUartError(UART_HandleTypeDef *huart);
/** 获取因接收队列满或 DMA 启动失败而丢弃的批次数。 */
uint32_t CommTuner_GetDroppedCount(void);

#ifdef __cplusplus
}
#endif

#endif
