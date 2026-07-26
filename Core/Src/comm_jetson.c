#include "comm_jetson.h"

#include <string.h>

#define COMM_JETSON_SYNC0 ((uint8_t)0x5AU)
#define COMM_JETSON_SYNC1 ((uint8_t)0xA5U)
#define COMM_JETSON_CMD_COLOR_START ((uint8_t)0x01U)
#define COMM_JETSON_CMD_CIRCLE_START ((uint8_t)0x02U)
#define COMM_JETSON_CMD_DISK_START ((uint8_t)0x03U)
#define COMM_JETSON_CMD_STOP ((uint8_t)0x04U)
#define COMM_JETSON_CMD_QR_START ((uint8_t)0x05U)
#define COMM_JETSON_CMD_ACK ((uint8_t)0x80U)
#define COMM_JETSON_CMD_COLOR_RESULT ((uint8_t)0x81U)
#define COMM_JETSON_CMD_CIRCLE_RESULT ((uint8_t)0x82U)
#define COMM_JETSON_CMD_DISK_RESULT ((uint8_t)0x83U)
#define COMM_JETSON_CMD_QR_RESULT ((uint8_t)0x84U)
#define COMM_JETSON_DISK_CENTER_NO_TARGET ((uint8_t)0U)
#define COMM_JETSON_DISK_CENTER_OK ((uint8_t)1U)
#define COMM_JETSON_RX_DMA_SIZE ((uint16_t)128U)
#define COMM_JETSON_FRAME_BUFFER_SIZE ((uint16_t)128U)

static UART_HandleTypeDef *g_uart;
static uint8_t g_rx_dma[COMM_JETSON_RX_DMA_SIZE];
static uint8_t g_tx_frame[16U];
static uint16_t g_rx_last_pos;
static uint8_t g_frame_buffer[COMM_JETSON_FRAME_BUFFER_SIZE];
static uint16_t g_frame_size;
static uint8_t g_session;
static uint8_t g_active;
static uint8_t g_mode;
static Detect_TargetList_t g_targets;
static Detect_DiskCenter_t g_disk_center;
static char g_qr_code[DETECT_QR_CODE_LENGTH + 1U];
static volatile uint8_t g_targets_new;
static volatile uint8_t g_disk_center_new;
static volatile uint8_t g_qr_new;
static volatile uint8_t g_tx_busy;
static volatile uint8_t g_ack_new;
static volatile uint8_t g_ack_command;
static volatile Detect_Status_t g_ack_status;
static volatile uint32_t g_last_result_tick;
static volatile uint8_t g_qr_waiting;
static volatile Detect_Status_t g_qr_wait_status;
static uint32_t g_qr_wait_started_tick;

static uint16_t CommJetson_Crc16(const uint8_t *data, uint16_t size)
{
  uint16_t crc = 0xFFFFU;
  uint16_t i;
  uint8_t bit;

  for (i = 0U; i < size; ++i)
  {
    crc ^= data[i];
    for (bit = 0U; bit < 8U; ++bit)
    {
      crc = ((crc & 1U) != 0U) ? (uint16_t)((crc >> 1U) ^ 0xA001U) : (uint16_t)(crc >> 1U);
    }
  }
  return crc;
}

static void CommJetson_StartRx(void)
{
  if (g_uart != NULL)
  {
    (void)HAL_UARTEx_ReceiveToIdle_DMA(g_uart, g_rx_dma, COMM_JETSON_RX_DMA_SIZE);
    if (g_uart->hdmarx != NULL)
    {
      __HAL_DMA_DISABLE_IT(g_uart->hdmarx, DMA_IT_HT);
    }
    g_rx_last_pos = 0U;
  }
}

static Detect_Status_t CommJetson_Send(uint8_t command, const uint8_t *payload, uint8_t length)
{
  uint16_t crc;
  uint16_t frame_size;

  if ((g_uart == NULL) || ((length > 0U) && (payload == NULL)))
  {
    return DETECT_STATUS_UART_ERROR;
  }
  if (g_tx_busy != 0U)
  {
    return DETECT_STATUS_BUSY;
  }

  g_tx_frame[0] = COMM_JETSON_SYNC0;
  g_tx_frame[1] = COMM_JETSON_SYNC1;
  g_tx_frame[2] = command;
  g_tx_frame[3] = g_session;
  g_tx_frame[4] = length;
  if (length > 0U)
  {
    memcpy(&g_tx_frame[5], payload, length);
  }
  crc = CommJetson_Crc16(&g_tx_frame[2], (uint16_t)(3U + length));
  g_tx_frame[5U + length] = (uint8_t)(crc & 0xFFU);
  g_tx_frame[6U + length] = (uint8_t)(crc >> 8U);
  frame_size = (uint16_t)(7U + length);
  g_tx_busy = 1U;
  if (HAL_UART_Transmit_DMA(g_uart, g_tx_frame, frame_size) != HAL_OK)
  {
    g_tx_busy = 0U;
    return DETECT_STATUS_UART_ERROR;
  }
  return DETECT_STATUS_OK;
}

static void CommJetson_ClearResults(void)
{
  g_targets = (Detect_TargetList_t){0};
  memset(&g_disk_center, 0, sizeof(g_disk_center));
  g_disk_center.status = DETECT_STATUS_NO_TARGET;
  memset(g_qr_code, 0, sizeof(g_qr_code));
  g_targets_new = 0U;
  g_disk_center_new = 0U;
  g_qr_new = 0U;
  g_last_result_tick = 0U;
}

static void CommJetson_ClearAck(void)
{
  g_ack_new = 0U;
  g_ack_command = 0U;
  g_ack_status = DETECT_STATUS_OK;
}

static Detect_Status_t CommJetson_Start(uint8_t command)
{
  uint8_t payload[2];
  uint8_t previous_session = g_session;
  Detect_Status_t status;

  ++g_session;
  payload[0] = (uint8_t)(DETECT_DEFAULT_PERIOD_MS & 0xFFU);
  payload[1] = (uint8_t)(DETECT_DEFAULT_PERIOD_MS >> 8U);
  status = CommJetson_Send(command, payload, sizeof(payload));
  if (status == DETECT_STATUS_OK)
  {
    g_active = 1U;
    g_mode = command;
    CommJetson_ClearResults();
  }
  else
  {
    g_session = previous_session;
  }
  return status;
}

static int16_t CommJetson_ReadI16(const uint8_t *data)
{
  return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static Detect_Status_t CommJetson_MapAckStatus(uint8_t status)
{
  switch (status)
  {
    case 0U:
      return DETECT_STATUS_OK;
    case 1U:
      return DETECT_STATUS_BAD_COMMAND;
    case 2U:
      return DETECT_STATUS_BAD_LENGTH;
    case 3U:
      return DETECT_STATUS_BAD_PERIOD;
    default:
      return DETECT_STATUS_BAD_COMMAND;
  }
}

static void CommJetson_HandleAck(const uint8_t *payload, uint8_t length)
{
  if (length != 2U)
  {
    return;
  }
  g_ack_command = payload[0];
  g_ack_status = CommJetson_MapAckStatus(payload[1]);
  g_ack_new = 1U;
}

static void CommJetson_HandleTargets(const uint8_t *payload, uint8_t length)
{
  uint8_t count;
  uint8_t i;

  if ((length < 1U) || (payload[0] > DETECT_TARGET_MAX) ||
      (length != (uint8_t)(1U + (payload[0] * 8U))))
  {
    return;
  }
  count = payload[0];
  g_targets.count = count;
  for (i = 0U; i < count; ++i)
  {
    const uint8_t *item = &payload[1U + (i * 8U)];
    g_targets.targets[i].type = item[0];
    g_targets.targets[i].x = CommJetson_ReadI16(&item[1]);
    g_targets.targets[i].y = CommJetson_ReadI16(&item[3]);
    g_targets.targets[i].confidence = item[5];
    g_targets.targets[i].measured = item[6];
    g_targets.targets[i].support_count = item[7];
  }
  g_targets_new = 1U;
  g_last_result_tick = HAL_GetTick();
}

static void CommJetson_HandleDiskCenter(const uint8_t *payload, uint8_t length)
{
  if (length != 7U)
  {
    return;
  }
  g_disk_center.status = (payload[0] == COMM_JETSON_DISK_CENTER_OK)
                           ? DETECT_STATUS_OK
                           : DETECT_STATUS_NO_TARGET;
  g_disk_center.x = CommJetson_ReadI16(&payload[1]);
  g_disk_center.y = CommJetson_ReadI16(&payload[3]);
  g_disk_center.support_count = payload[5];
  g_disk_center.measured_count = payload[6];
  if (g_disk_center.support_count == 0U)
  {
    g_disk_center.status = DETECT_STATUS_NO_TARGET;
    g_disk_center.x = 0;
    g_disk_center.y = 0;
  }
  g_disk_center_new = 1U;
  g_last_result_tick = HAL_GetTick();
}

static void CommJetson_HandleQr(const uint8_t *payload, uint8_t length)
{
  if (length != DETECT_QR_CODE_LENGTH)
  {
    return;
  }
  memcpy(g_qr_code, payload, DETECT_QR_CODE_LENGTH);
  g_qr_code[DETECT_QR_CODE_LENGTH] = '\0';
  g_qr_new = 1U;
  g_last_result_tick = HAL_GetTick();
}

static void CommJetson_HandleFrame(const uint8_t *frame, uint16_t frame_size)
{
  uint8_t command = frame[2];
  uint8_t length = frame[4];
  const uint8_t *payload = &frame[5];
  uint16_t received_crc = (uint16_t)frame[frame_size - 2U] | ((uint16_t)frame[frame_size - 1U] << 8U);

  if ((CommJetson_Crc16(&frame[2], (uint16_t)(3U + length)) != received_crc) || (frame[3] != g_session))
  {
    return;
  }
  if (command == COMM_JETSON_CMD_ACK)
  {
    CommJetson_HandleAck(payload, length);
    return;
  }
  if (g_active == 0U)
  {
    return;
  }
  if (((command == COMM_JETSON_CMD_COLOR_RESULT) && (g_mode == COMM_JETSON_CMD_COLOR_START)) ||
      ((command == COMM_JETSON_CMD_CIRCLE_RESULT) && (g_mode == COMM_JETSON_CMD_CIRCLE_START)))
  {
    CommJetson_HandleTargets(payload, length);
  }
  else if ((command == COMM_JETSON_CMD_DISK_RESULT) && (g_mode == COMM_JETSON_CMD_DISK_START))
  {
    CommJetson_HandleDiskCenter(payload, length);
  }
  else if ((command == COMM_JETSON_CMD_QR_RESULT) && (g_mode == COMM_JETSON_CMD_QR_START))
  {
    CommJetson_HandleQr(payload, length);
  }
}

static void CommJetson_Parse(void)
{
  uint16_t total;

  while (g_frame_size >= 2U)
  {
    if ((g_frame_buffer[0] != COMM_JETSON_SYNC0) || (g_frame_buffer[1] != COMM_JETSON_SYNC1))
    {
      memmove(g_frame_buffer, &g_frame_buffer[1], --g_frame_size);
      continue;
    }
    if (g_frame_size < 5U)
    {
      return;
    }
    total = (uint16_t)(7U + g_frame_buffer[4]);
    if (total > COMM_JETSON_FRAME_BUFFER_SIZE)
    {
      memmove(g_frame_buffer, &g_frame_buffer[1], --g_frame_size);
      continue;
    }
    if (g_frame_size < total)
    {
      return;
    }
    CommJetson_HandleFrame(g_frame_buffer, total);
    g_frame_size = (uint16_t)(g_frame_size - total);
    if (g_frame_size > 0U)
    {
      memmove(g_frame_buffer, &g_frame_buffer[total], g_frame_size);
    }
  }
}

void CommJetson_Init(UART_HandleTypeDef *huart)
{
  g_uart = huart;
  g_frame_size = 0U;
  g_rx_last_pos = 0U;
  g_session = 0U;
  g_active = 0U;
  g_mode = 0U;
  g_tx_busy = 0U;
  g_qr_waiting = 0U;
  g_qr_wait_status = DETECT_STATUS_OK;
  g_qr_wait_started_tick = 0U;
  CommJetson_ClearResults();
  CommJetson_ClearAck();
  CommJetson_StartRx();
}

void CommJetson_Update(void)
{
  if (g_qr_waiting == 0U)
  {
    return;
  }

  if (g_qr_new != 0U)
  {
    g_qr_wait_status = DETECT_STATUS_OK;
    g_qr_waiting = 0U;
    return;
  }

  if ((g_ack_new != 0U) && (g_ack_command == COMM_JETSON_CMD_QR_START))
  {
    Detect_Status_t ack_status = g_ack_status;

    g_ack_new = 0U;
    if (ack_status != DETECT_STATUS_OK)
    {
      g_qr_wait_status = ack_status;
      g_qr_waiting = 0U;
      return;
    }
  }

  if ((HAL_GetTick() - g_qr_wait_started_tick) >= DETECT_QR_TIMEOUT_MS)
  {
    g_qr_wait_status = DETECT_STATUS_TIMEOUT;
    g_qr_waiting = 0U;
  }
}

void CommJetson_OnUartRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
  uint16_t current_pos;
  uint16_t copy_size;

  if (huart != g_uart)
  {
    return;
  }
  current_pos = (size > COMM_JETSON_RX_DMA_SIZE) ? COMM_JETSON_RX_DMA_SIZE : size;
  if (current_pos == g_rx_last_pos)
  {
    return;
  }
  if (current_pos > g_rx_last_pos)
  {
    copy_size = (uint16_t)(current_pos - g_rx_last_pos);
    if (copy_size > (uint16_t)(COMM_JETSON_FRAME_BUFFER_SIZE - g_frame_size))
    {
      g_frame_size = 0U;
    }
    memcpy(&g_frame_buffer[g_frame_size], &g_rx_dma[g_rx_last_pos], copy_size);
    g_frame_size = (uint16_t)(g_frame_size + copy_size);
  }
  else
  {
    copy_size = (uint16_t)(COMM_JETSON_RX_DMA_SIZE - g_rx_last_pos);
    if (copy_size > (uint16_t)(COMM_JETSON_FRAME_BUFFER_SIZE - g_frame_size))
    {
      g_frame_size = 0U;
    }
    memcpy(&g_frame_buffer[g_frame_size], &g_rx_dma[g_rx_last_pos], copy_size);
    g_frame_size = (uint16_t)(g_frame_size + copy_size);
    if (current_pos > 0U)
    {
      if (current_pos > (uint16_t)(COMM_JETSON_FRAME_BUFFER_SIZE - g_frame_size))
      {
        g_frame_size = 0U;
      }
      memcpy(&g_frame_buffer[g_frame_size], g_rx_dma, current_pos);
      g_frame_size = (uint16_t)(g_frame_size + current_pos);
    }
  }
  g_rx_last_pos = (current_pos == COMM_JETSON_RX_DMA_SIZE) ? 0U : current_pos;
  CommJetson_Parse();
}

void CommJetson_OnUartError(UART_HandleTypeDef *huart)
{
  if (huart == g_uart)
  {
    g_frame_size = 0U;
    CommJetson_StartRx();
  }
}

void CommJetson_OnUartTxComplete(UART_HandleTypeDef *huart)
{
  if (huart == g_uart)
  {
    g_tx_busy = 0U;
  }
}

Detect_Status_t detect_color_start(void) { return CommJetson_Start(COMM_JETSON_CMD_COLOR_START); }
Detect_Status_t detect_circle_start(void) { return CommJetson_Start(COMM_JETSON_CMD_CIRCLE_START); }
Detect_Status_t detect_disk_center_start(void) { return CommJetson_Start(COMM_JETSON_CMD_DISK_START); }
Detect_Status_t detect_qr_start(void) { return CommJetson_Start(COMM_JETSON_CMD_QR_START); }

Detect_Status_t detect_stop(void)
{
  Detect_Status_t status = CommJetson_Send(COMM_JETSON_CMD_STOP, NULL, 0U);

  g_active = 0U;
  g_mode = 0U;
  CommJetson_ClearResults();
  return status;
}

uint8_t detect_get_targets(Detect_TargetList_t *result)
{
  if ((result == NULL) || (g_targets_new == 0U))
  {
    return 0U;
  }
  *result = g_targets;
  g_targets_new = 0U;
  return 1U;
}

uint8_t detect_get_disk_center(Detect_DiskCenter_t *result)
{
  if ((result == NULL) || (g_disk_center_new == 0U))
  {
    return 0U;
  }
  *result = g_disk_center;
  g_disk_center_new = 0U;
  return 1U;
}

uint8_t detect_get_qr(char code[DETECT_QR_CODE_LENGTH + 1U])
{
  if ((code == NULL) || (g_qr_new == 0U))
  {
    return 0U;
  }
  memcpy(code, g_qr_code, sizeof(g_qr_code));
  g_qr_new = 0U;
  return 1U;
}

Detect_Status_t detect_qr_read_blocking(char code[DETECT_QR_CODE_LENGTH + 1U])
{
  Detect_Status_t status;

  if (code == NULL)
  {
    return DETECT_STATUS_BAD_PARAMETER;
  }
  code[0] = '\0';
  CommJetson_ClearResults();
  CommJetson_ClearAck();
  status = detect_qr_start();
  if (status != DETECT_STATUS_OK)
  {
    return status;
  }

  g_qr_wait_started_tick = HAL_GetTick();
  g_qr_wait_status = DETECT_STATUS_BUSY;
  g_qr_waiting = 1U;
  while (g_qr_waiting != 0U)
  {
    __WFI();
  }

  status = g_qr_wait_status;
  if ((status != DETECT_STATUS_OK) || (detect_get_qr(code) == 0U))
  {
    code[0] = '\0';
    if (status == DETECT_STATUS_OK)
    {
      status = DETECT_STATUS_UART_ERROR;
    }
  }
  (void)detect_stop();
  return status;
}

uint8_t detect_is_active(void) { return g_active; }

uint8_t detect_is_fresh(uint32_t timeout_ms)
{
  return ((g_last_result_tick != 0U) && ((HAL_GetTick() - g_last_result_tick) <= timeout_ms)) ? 1U : 0U;
}
