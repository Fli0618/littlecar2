#include "comm_tuner.h"

#include <string.h>

#define COMM_TUNER_RX_DMA_SIZE ((uint16_t)128U)
#define COMM_TUNER_QUEUE_DEPTH ((uint8_t)4U)

typedef struct
{
  uint16_t length;
  uint8_t data[COMM_TUNER_RX_DMA_SIZE];
} CommTuner_Message_t;

static UART_HandleTypeDef *g_uart;
static uint8_t g_rx_dma[COMM_TUNER_RX_DMA_SIZE];
static uint16_t g_rx_last_pos;
static CommTuner_Message_t g_rx_queue[COMM_TUNER_QUEUE_DEPTH];
static volatile uint8_t g_rx_queue_head;
static volatile uint8_t g_rx_queue_tail;
static volatile uint8_t g_rx_queue_count;
static volatile uint32_t g_rx_dropped_count;
static uint8_t g_tx_dma[COMM_TUNER_RX_DMA_SIZE];
static volatile uint8_t g_tx_busy;

static uint8_t CommTuner_NextQueueIndex(uint8_t index)
{
  return (uint8_t)((index + 1U) % COMM_TUNER_QUEUE_DEPTH);
}

static HAL_StatusTypeDef CommTuner_StartRx(void)
{
  HAL_StatusTypeDef status;

  status = HAL_UARTEx_ReceiveToIdle_DMA(g_uart, g_rx_dma, COMM_TUNER_RX_DMA_SIZE);
  if ((status == HAL_OK) && (g_uart->hdmarx != NULL))
  {
    __HAL_DMA_DISABLE_IT(g_uart->hdmarx, DMA_IT_HT);
  }
  return status;
}

static void CommTuner_ClearReceiveQueue(void)
{
  g_rx_queue_head = 0U;
  g_rx_queue_tail = 0U;
  g_rx_queue_count = 0U;
  g_rx_last_pos = 0U;
}

static void CommTuner_QueueReceivedBytes(uint16_t current_pos)
{
  CommTuner_Message_t *message;
  uint16_t first_size;
  uint16_t second_size;

  if (current_pos == g_rx_last_pos)
  {
    return;
  }

  if (g_rx_queue_count >= COMM_TUNER_QUEUE_DEPTH)
  {
    ++g_rx_dropped_count;
    g_rx_last_pos = (current_pos == COMM_TUNER_RX_DMA_SIZE) ? 0U : current_pos;
    return;
  }

  message = &g_rx_queue[g_rx_queue_head];
  if (current_pos > g_rx_last_pos)
  {
    first_size = (uint16_t)(current_pos - g_rx_last_pos);
    memcpy(message->data, &g_rx_dma[g_rx_last_pos], first_size);
    message->length = first_size;
  }
  else
  {
    first_size = (uint16_t)(COMM_TUNER_RX_DMA_SIZE - g_rx_last_pos);
    second_size = current_pos;
    memcpy(message->data, &g_rx_dma[g_rx_last_pos], first_size);
    if (second_size > 0U)
    {
      memcpy(&message->data[first_size], g_rx_dma, second_size);
    }
    message->length = (uint16_t)(first_size + second_size);
  }

  g_rx_last_pos = (current_pos == COMM_TUNER_RX_DMA_SIZE) ? 0U : current_pos;
  g_rx_queue_head = CommTuner_NextQueueIndex(g_rx_queue_head);
  ++g_rx_queue_count;
}

HAL_StatusTypeDef CommTuner_Init(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return HAL_ERROR;
  }

  g_uart = huart;
  g_tx_busy = 0U;
  g_rx_dropped_count = 0U;
  CommTuner_ClearReceiveQueue();
  return CommTuner_StartRx();
}

void CommTuner_Process(void)
{
  CommTuner_Message_t *message;
  uint16_t tx_length;

  if ((g_uart == NULL) || (g_tx_busy != 0U) || (g_rx_queue_count == 0U))
  {
    return;
  }

  message = &g_rx_queue[g_rx_queue_tail];
  tx_length = message->length;
  memcpy(g_tx_dma, message->data, tx_length);
  g_rx_queue_tail = CommTuner_NextQueueIndex(g_rx_queue_tail);
  --g_rx_queue_count;

  g_tx_busy = 1U;
  if (HAL_UART_Transmit_DMA(g_uart, g_tx_dma, tx_length) != HAL_OK)
  {
    g_tx_busy = 0U;
    ++g_rx_dropped_count;
  }
}

void CommTuner_OnUartRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
  uint16_t current_pos;

  if (huart != g_uart)
  {
    return;
  }

  current_pos = (size > COMM_TUNER_RX_DMA_SIZE) ? COMM_TUNER_RX_DMA_SIZE : size;
  CommTuner_QueueReceivedBytes(current_pos);
}

void CommTuner_OnUartTxComplete(UART_HandleTypeDef *huart)
{
  if (huart == g_uart)
  {
    g_tx_busy = 0U;
  }
}

void CommTuner_OnUartError(UART_HandleTypeDef *huart)
{
  if (huart == g_uart)
  {
    g_tx_busy = 0U;
    CommTuner_ClearReceiveQueue();
    (void)CommTuner_StartRx();
  }
}

uint32_t CommTuner_GetDroppedCount(void)
{
  return g_rx_dropped_count;
}
