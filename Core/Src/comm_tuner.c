#include "comm_tuner.h"

#include <string.h>
#include <math.h>

#include "advance_chassis.h"
#include "advance_motion.h"

#define COMM_TUNER_RX_DMA_SIZE ((uint16_t)128U)
#define COMM_TUNER_QUEUE_DEPTH ((uint8_t)4U)
#define COMM_TUNER_PROTOCOL_VERSION ((uint8_t)2U)
#define COMM_TUNER_MAX_PAYLOAD_SIZE ((uint16_t)96U)
#define COMM_TUNER_FRAME_OVERHEAD ((uint16_t)9U)
#define COMM_TUNER_FRAME_MAX_SIZE (COMM_TUNER_FRAME_OVERHEAD + COMM_TUNER_MAX_PAYLOAD_SIZE)
#define COMM_TUNER_HEARTBEAT_TIMEOUT_MS ((uint32_t)1500U)
#define COMM_TUNER_TELEMETRY_PERIOD_MS ((uint32_t)40U)
#define COMM_TUNER_PATH_TELEMETRY_PERIOD_MS ((uint32_t)50U)
#define COMM_TUNER_GOTO_VMAX_MM_S (1500.0f)
#define COMM_TUNER_GOTO_WMAX_DEG_S (120.0f)
#define COMM_TUNER_GOTO_TIMEOUT_MS ((uint32_t)15000U)

/* Telemetry payload bytes 14..15: remote link state plus heartbeat age in ms. */
#define COMM_TUNER_LINK_STATUS_ACTIVE ((uint16_t)0x8000U)
#define COMM_TUNER_LINK_STATUS_TIMEOUT ((uint16_t)0x4000U)
#define COMM_TUNER_LINK_STATUS_AGE_MASK ((uint16_t)0x3FFFU)

#define COMM_TUNER_SYNC0 ((uint8_t)0xA5U)
#define COMM_TUNER_SYNC1 ((uint8_t)0x5AU)

#define COMM_TUNER_CMD_GET_PID ((uint8_t)0x01U)
#define COMM_TUNER_CMD_SET_PID ((uint8_t)0x02U)
#define COMM_TUNER_CMD_RESTORE_PID ((uint8_t)0x03U)
#define COMM_TUNER_CMD_GOTO_POSE ((uint8_t)0x10U)
#define COMM_TUNER_CMD_STOP ((uint8_t)0x11U)
#define COMM_TUNER_CMD_HEARTBEAT ((uint8_t)0x12U)
#define COMM_TUNER_CMD_SET_YAW_SOURCE ((uint8_t)0x13U)
#define COMM_TUNER_CMD_RESET_ORIGIN ((uint8_t)0x14U)
#define COMM_TUNER_CMD_GET_GOTO_STRATEGY ((uint8_t)0x15U)
#define COMM_TUNER_CMD_SET_GOTO_STRATEGY ((uint8_t)0x16U)
#define COMM_TUNER_CMD_PATH_BEGIN ((uint8_t)0x20U)
#define COMM_TUNER_CMD_PATH_CHUNK ((uint8_t)0x21U)
#define COMM_TUNER_CMD_PATH_COMMIT ((uint8_t)0x22U)
#define COMM_TUNER_CMD_PATH_START ((uint8_t)0x23U)
#define COMM_TUNER_CMD_PATH_ABORT ((uint8_t)0x24U)
#define COMM_TUNER_CMD_PATH_STATUS ((uint8_t)0x25U)
#define COMM_TUNER_CMD_GET_PATH_CONFIG ((uint8_t)0x26U)
#define COMM_TUNER_CMD_SET_PATH_CONFIG ((uint8_t)0x27U)
#define COMM_TUNER_CMD_RESTORE_PATH_CONFIG ((uint8_t)0x28U)
#define COMM_TUNER_CMD_ACK ((uint8_t)0x80U)
#define COMM_TUNER_CMD_PID ((uint8_t)0x81U)
#define COMM_TUNER_CMD_TELEMETRY ((uint8_t)0x82U)
#define COMM_TUNER_CMD_GOTO_STRATEGY ((uint8_t)0x83U)
#define COMM_TUNER_CMD_PATH_STATUS_RESPONSE ((uint8_t)0x84U)
#define COMM_TUNER_CMD_PATH_TELEMETRY ((uint8_t)0x85U)
#define COMM_TUNER_CMD_PATH_CONFIG ((uint8_t)0x86U)
#define COMM_TUNER_CMD_ERROR ((uint8_t)0xE0U)

#define COMM_TUNER_ERROR_BAD_CRC ((uint8_t)0x01U)
#define COMM_TUNER_ERROR_BAD_VERSION ((uint8_t)0x02U)
#define COMM_TUNER_ERROR_BAD_LENGTH ((uint8_t)0x03U)
#define COMM_TUNER_ERROR_BAD_COMMAND ((uint8_t)0x04U)
#define COMM_TUNER_ERROR_BAD_PID ((uint8_t)0x05U)
#define COMM_TUNER_ERROR_BAD_GOAL ((uint8_t)0x06U)
#define COMM_TUNER_ERROR_BUSY ((uint8_t)0x07U)
#define COMM_TUNER_ERROR_NO_ORIGIN ((uint8_t)0x08U)
#define COMM_TUNER_ERROR_NO_POSE ((uint8_t)0x09U)
#define COMM_TUNER_ERROR_POSE_TIMEOUT ((uint8_t)0x0AU)
#define COMM_TUNER_ERROR_BAD_PATH_CONFIG ((uint8_t)0x0BU)

#define COMM_TUNER_SET_PID_PAYLOAD_SIZE ((uint16_t)24U)
#define COMM_TUNER_PID_PAYLOAD_SIZE ((uint16_t)28U)
#define COMM_TUNER_GOTO_PAYLOAD_SIZE ((uint16_t)25U)
#define COMM_TUNER_TELEMETRY_PAYLOAD_SIZE ((uint16_t)96U)
#define COMM_TUNER_PATH_POINT_BYTES ((uint16_t)12U)
#define COMM_TUNER_PATH_CHUNK_HEADER_BYTES ((uint16_t)7U)
#define COMM_TUNER_PATH_CHUNK_MAX_POINTS ((uint8_t)7U)
#define COMM_TUNER_PATH_STATUS_PAYLOAD_SIZE ((uint16_t)17U)
#define COMM_TUNER_PATH_TELEMETRY_PAYLOAD_SIZE ((uint16_t)50U)
#define COMM_TUNER_SET_PATH_CONFIG_PAYLOAD_SIZE ((uint16_t)56U)
#define COMM_TUNER_PATH_CONFIG_PAYLOAD_SIZE ((uint16_t)60U)

typedef enum
{
  COMM_TUNER_PATH_STAGING_EMPTY = 0,
  COMM_TUNER_PATH_STAGING_RECEIVING
} CommTuner_PathStagingState_t;

typedef struct
{
  uint16_t length;
  uint8_t data[COMM_TUNER_RX_DMA_SIZE];
} CommTuner_RxMessage_t;

typedef struct
{
  uint16_t length;
  uint8_t data[COMM_TUNER_FRAME_MAX_SIZE];
} CommTuner_TxFrame_t;

static UART_HandleTypeDef *g_uart;
static uint8_t g_rx_dma[COMM_TUNER_RX_DMA_SIZE];
static uint16_t g_rx_last_pos;
static CommTuner_RxMessage_t g_rx_queue[COMM_TUNER_QUEUE_DEPTH];
static volatile uint8_t g_rx_queue_head;
static volatile uint8_t g_rx_queue_tail;
static volatile uint8_t g_rx_queue_count;
static uint8_t g_parse_buffer[COMM_TUNER_FRAME_MAX_SIZE];
static uint16_t g_parse_size;
static CommTuner_TxFrame_t g_tx_queue[COMM_TUNER_QUEUE_DEPTH];
static uint8_t g_tx_queue_head;
static uint8_t g_tx_queue_tail;
static uint8_t g_tx_queue_count;
static uint8_t g_tx_dma[COMM_TUNER_FRAME_MAX_SIZE];
static volatile uint8_t g_tx_busy;
static uint8_t g_response_cache_valid;
static uint8_t g_response_cache_command;
static uint8_t g_response_cache_sequence;
static CommTuner_TxFrame_t g_response_cache;
static CommTuner_TxFrame_t g_telemetry_frame;
static volatile uint8_t g_telemetry_pending;
static CommTuner_TxFrame_t g_path_telemetry_frame;
static volatile uint8_t g_path_telemetry_pending;
static uint8_t g_telemetry_sequence;
static volatile uint32_t g_telemetry_dropped_count;
static volatile uint8_t g_remote_goal_active;
static volatile uint8_t g_remote_heartbeat_timeout;
static volatile uint32_t g_last_heartbeat_tick;
static volatile uint32_t g_last_telemetry_tick;
static volatile uint32_t g_last_path_telemetry_tick;
static volatile uint32_t g_rx_dropped_count;
static AdvanceMotion_PathPoint_t g_path_buffer_a[ADVANCE_MOTION_PATH_MAX_POINTS];
static AdvanceMotion_PathPoint_t g_path_buffer_b[ADVANCE_MOTION_PATH_MAX_POINTS];
static AdvanceMotion_PathPoint_t *g_path_active_points = g_path_buffer_a;
static AdvanceMotion_PathPoint_t *g_path_staging_points = g_path_buffer_b;
static uint8_t g_path_received[(ADVANCE_MOTION_PATH_MAX_POINTS + 7U) / 8U];
static uint32_t g_path_active_id;
static uint32_t g_path_staging_id;
static uint16_t g_path_active_count;
static uint16_t g_path_staging_count;
static uint16_t g_path_staging_received_count;
static uint16_t g_path_staging_crc;
static CommTuner_PathStagingState_t g_path_staging_state;

static uint8_t CommTuner_NextQueueIndex(uint8_t index)
{
  return (uint8_t)((index + 1U) % COMM_TUNER_QUEUE_DEPTH);
}

static uint16_t CommTuner_Crc16(const uint8_t *data, uint16_t size)
{
  uint16_t crc = 0xFFFFU;
  uint16_t index;
  uint8_t bit;

  for (index = 0U; index < size; ++index)
  {
    crc ^= (uint16_t)data[index] << 8U;
    for (bit = 0U; bit < 8U; ++bit)
    {
      crc = ((crc & 0x8000U) != 0U) ? (uint16_t)((crc << 1U) ^ 0x1021U) : (uint16_t)(crc << 1U);
    }
  }
  return crc;
}

static uint32_t CommTuner_ReadU32(const uint8_t *data)
{
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8U) |
         ((uint32_t)data[2] << 16U) |
         ((uint32_t)data[3] << 24U);
}

static void CommTuner_WriteU32(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8U);
  data[2] = (uint8_t)(value >> 16U);
  data[3] = (uint8_t)(value >> 24U);
}

static void CommTuner_WriteU16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8U);
}

static uint16_t CommTuner_GetRemoteLinkStatus(uint32_t now_tick)
{
  uint32_t heartbeat_age_ms;
  uint16_t status = 0U;

  if (g_remote_goal_active != 0U)
  {
    heartbeat_age_ms = now_tick - g_last_heartbeat_tick;
    if (heartbeat_age_ms > COMM_TUNER_LINK_STATUS_AGE_MASK)
    {
      heartbeat_age_ms = COMM_TUNER_LINK_STATUS_AGE_MASK;
    }
    status = (uint16_t)(COMM_TUNER_LINK_STATUS_ACTIVE | (uint16_t)heartbeat_age_ms);
  }
  if (g_remote_heartbeat_timeout != 0U)
  {
    status |= COMM_TUNER_LINK_STATUS_TIMEOUT;
  }
  return status;
}

static float CommTuner_ReadFloat(const uint8_t *data)
{
  float value;

  memcpy(&value, data, sizeof(value));
  return value;
}

static void CommTuner_WriteFloat(uint8_t *data, float value)
{
  memcpy(data, &value, sizeof(value));
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

static void CommTuner_ClearReceiveState(void)
{
  g_rx_queue_head = 0U;
  g_rx_queue_tail = 0U;
  g_rx_queue_count = 0U;
  g_rx_last_pos = 0U;
  g_parse_size = 0U;
}

static void CommTuner_ClearTransmitState(void)
{
  g_tx_queue_head = 0U;
  g_tx_queue_tail = 0U;
  g_tx_queue_count = 0U;
  g_tx_busy = 0U;
  g_telemetry_pending = 0U;
  g_path_telemetry_pending = 0U;
}

static void CommTuner_ClearTelemetry(void)
{
  g_telemetry_pending = 0U;
  g_path_telemetry_pending = 0U;
}

static uint8_t CommTuner_QueueTransmit(const uint8_t *data, uint16_t length)
{
  CommTuner_TxFrame_t *frame;

  if ((data == NULL) || (length == 0U) || (length > COMM_TUNER_FRAME_MAX_SIZE) ||
      (g_tx_queue_count >= COMM_TUNER_QUEUE_DEPTH))
  {
    ++g_rx_dropped_count;
    return 0U;
  }

  frame = &g_tx_queue[g_tx_queue_head];
  memcpy(frame->data, data, length);
  frame->length = length;
  g_tx_queue_head = CommTuner_NextQueueIndex(g_tx_queue_head);
  ++g_tx_queue_count;
  return 1U;
}

static uint16_t CommTuner_BuildFrame(uint8_t command, uint8_t sequence,
                                      const uint8_t *payload, uint16_t payload_length,
                                      uint8_t frame[COMM_TUNER_FRAME_MAX_SIZE])
{
  uint16_t crc;
  uint16_t total_length;

  frame[0] = COMM_TUNER_SYNC0;
  frame[1] = COMM_TUNER_SYNC1;
  frame[2] = COMM_TUNER_PROTOCOL_VERSION;
  frame[3] = command;
  frame[4] = sequence;
  frame[5] = (uint8_t)payload_length;
  frame[6] = (uint8_t)(payload_length >> 8U);
  if (payload_length > 0U)
  {
    memcpy(&frame[7], payload, payload_length);
  }
  crc = CommTuner_Crc16(&frame[2], (uint16_t)(5U + payload_length));
  frame[7U + payload_length] = (uint8_t)crc;
  frame[8U + payload_length] = (uint8_t)(crc >> 8U);
  total_length = (uint16_t)(COMM_TUNER_FRAME_OVERHEAD + payload_length);
  return total_length;
}

static void CommTuner_SendResponse(uint8_t request_command, uint8_t request_sequence,
                                   uint8_t response_command, const uint8_t *payload,
                                   uint16_t payload_length, uint8_t cache_response)
{
  uint8_t frame[COMM_TUNER_FRAME_MAX_SIZE];
  uint16_t frame_length;

  frame_length = CommTuner_BuildFrame(response_command, request_sequence, payload,
                                      payload_length, frame);
  (void)CommTuner_QueueTransmit(frame, frame_length);
  if (cache_response != 0U)
  {
    g_response_cache.length = frame_length;
    memcpy(g_response_cache.data, frame, frame_length);
    g_response_cache_command = request_command;
    g_response_cache_sequence = request_sequence;
    g_response_cache_valid = 1U;
  }
}

static void CommTuner_SendAck(uint8_t request_command, uint8_t request_sequence,
                              uint32_t revision, uint8_t has_revision)
{
  uint8_t payload[5U];
  uint16_t payload_length = 1U;

  payload[0] = request_command;
  if (has_revision != 0U)
  {
    CommTuner_WriteU32(&payload[1], revision);
    payload_length = 5U;
  }
  CommTuner_SendResponse(request_command, request_sequence, COMM_TUNER_CMD_ACK,
                         payload, payload_length, 1U);
}

static void CommTuner_SendError(uint8_t request_command, uint8_t request_sequence,
                                uint8_t error_code, uint8_t cache_response)
{
  uint8_t payload[2U];

  payload[0] = request_command;
  payload[1] = error_code;
  CommTuner_SendResponse(request_command, request_sequence, COMM_TUNER_CMD_ERROR,
                         payload, sizeof(payload), cache_response);
}

static void CommTuner_SendPid(uint8_t request_command, uint8_t request_sequence)
{
  AdvanceMotion_PidConfig_t config;
  uint32_t revision;
  uint8_t payload[COMM_TUNER_PID_PAYLOAD_SIZE];

  if (AdvanceMotion_GetPidConfig(&config, &revision) != ADVANCE_MOTION_STATUS_OK)
  {
    CommTuner_SendError(request_command, request_sequence, COMM_TUNER_ERROR_BAD_PID, 1U);
    return;
  }

  CommTuner_WriteU32(&payload[0], revision);
  CommTuner_WriteFloat(&payload[4], config.kp_pos);
  CommTuner_WriteFloat(&payload[8], config.ki_pos);
  CommTuner_WriteFloat(&payload[12], config.kd_pos);
  CommTuner_WriteFloat(&payload[16], config.kp_yaw);
  CommTuner_WriteFloat(&payload[20], config.ki_yaw);
  CommTuner_WriteFloat(&payload[24], config.kd_yaw);
  CommTuner_SendResponse(request_command, request_sequence, COMM_TUNER_CMD_PID,
                         payload, sizeof(payload), 1U);
}

static void CommTuner_SendPathConfig(uint8_t request_command, uint8_t request_sequence)
{
  AdvanceMotion_PathControlConfig_t config;
  uint32_t revision;
  uint8_t payload[COMM_TUNER_PATH_CONFIG_PAYLOAD_SIZE];
  float values[14U];
  uint8_t index;

  if (AdvanceMotion_GetPathControlConfig(&config, &revision) != ADVANCE_MOTION_STATUS_OK)
  {
    CommTuner_SendError(request_command, request_sequence,
                        COMM_TUNER_ERROR_BAD_PATH_CONFIG, 1U);
    return;
  }
  CommTuner_WriteU32(&payload[0], revision);
  values[0] = config.kp_cross_track;
  values[1] = config.kd_cross_track_velocity;
  values[2] = config.kp_yaw;
  values[3] = config.kd_yaw_rate;
  values[4] = config.cruise_speed_mm_s;
  values[5] = config.max_yaw_rate_deg_s;
  values[6] = config.accel_mm_s2;
  values[7] = config.decel_mm_s2;
  values[8] = config.max_lateral_accel_mm_s2;
  values[9] = config.lookahead_min_mm;
  values[10] = config.lookahead_base_mm;
  values[11] = config.lookahead_speed_gain_s;
  values[12] = config.lookahead_curve_gain_mm;
  values[13] = config.lookahead_max_mm;
  for (index = 0U; index < 14U; ++index)
  {
    CommTuner_WriteFloat(&payload[4U + ((uint16_t)index * 4U)], values[index]);
  }
  CommTuner_SendResponse(request_command, request_sequence, COMM_TUNER_CMD_PATH_CONFIG,
                         payload, sizeof(payload), 1U);
}

static void CommTuner_SendGotoStrategy(uint8_t request_command, uint8_t request_sequence)
{
  uint8_t payload[1U];

  if (AdvanceMotion_GetLargeYawAlignEnabled(&payload[0]) != ADVANCE_MOTION_STATUS_OK)
  {
    CommTuner_SendError(request_command, request_sequence, COMM_TUNER_ERROR_BAD_GOAL, 1U);
    return;
  }
  CommTuner_SendResponse(request_command, request_sequence, COMM_TUNER_CMD_GOTO_STRATEGY,
                         payload, sizeof(payload), 1U);
}

static void CommTuner_ClearPathStaging(void)
{
  g_path_staging_id = 0U;
  g_path_staging_count = 0U;
  g_path_staging_received_count = 0U;
  g_path_staging_crc = 0U;
  g_path_staging_state = COMM_TUNER_PATH_STAGING_EMPTY;
  memset(g_path_received, 0, sizeof(g_path_received));
}

static uint8_t CommTuner_IsPathPointValid(const AdvanceMotion_PathPoint_t *point)
{
  return ((point != NULL) && (isfinite(point->x_mm) != 0) &&
          (isfinite(point->y_mm) != 0) && (isfinite(point->yaw_deg) != 0) &&
          (point->x_mm >= ADVANCE_MOTION_WORLD_X_MIN_MM) &&
          (point->x_mm <= ADVANCE_MOTION_WORLD_X_MAX_MM) &&
          (point->y_mm >= ADVANCE_MOTION_WORLD_Y_MIN_MM) &&
          (point->y_mm <= ADVANCE_MOTION_WORLD_Y_MAX_MM)) ? 1U : 0U;
}

static void CommTuner_SendPathStatus(uint8_t request_command, uint8_t request_sequence)
{
  AdvanceMotion_RuntimeStatus_t motion_status;
  uint8_t payload[COMM_TUNER_PATH_STATUS_PAYLOAD_SIZE] = {0};

  (void)AdvanceMotion_GetStatus(&motion_status);
  payload[0] = (uint8_t)motion_status.state;
  payload[1] = (g_path_active_count > 0U) ? 1U : 0U;
  payload[2] = (uint8_t)g_path_staging_state;
  CommTuner_WriteU16(&payload[3], g_path_active_count);
  CommTuner_WriteU16(&payload[5], g_path_staging_count);
  CommTuner_WriteU16(&payload[7], g_path_staging_received_count);
  CommTuner_WriteU32(&payload[9], g_path_active_id);
  CommTuner_WriteU32(&payload[13], g_path_staging_id);
  CommTuner_SendResponse(request_command, request_sequence,
                         COMM_TUNER_CMD_PATH_STATUS_RESPONSE, payload,
                         sizeof(payload), 1U);
}

static void CommTuner_QueuePathTelemetry(void)
{
  AdvanceMotion_DebugSnapshot_t snapshot;
  uint8_t payload[COMM_TUNER_PATH_TELEMETRY_PAYLOAD_SIZE];
  uint16_t frame_length;

  if (AdvanceMotion_GetDebugSnapshot(&snapshot) != ADVANCE_MOTION_STATUS_OK)
  {
    return;
  }
  CommTuner_WriteU32(&payload[0], snapshot.tick);
  payload[4] = (uint8_t)snapshot.state;
  CommTuner_WriteU16(&payload[5], snapshot.nearest_segment_index);
  CommTuner_WriteU16(&payload[7], snapshot.target_segment_index);
  CommTuner_WriteFloat(&payload[9], snapshot.path_progress_mm);
  CommTuner_WriteFloat(&payload[13], snapshot.path_remaining_mm);
  CommTuner_WriteFloat(&payload[17], snapshot.path_projection_x_mm);
  CommTuner_WriteFloat(&payload[21], snapshot.path_projection_y_mm);
  CommTuner_WriteFloat(&payload[25], snapshot.path_curvature_preview_1_mm);
  CommTuner_WriteFloat(&payload[29], snapshot.path_yaw_gradient_deg_per_mm);
  CommTuner_WriteFloat(&payload[33], snapshot.path_reference_speed_mm_s);
  CommTuner_WriteFloat(&payload[37], snapshot.path_lookahead_mm);
  CommTuner_WriteFloat(&payload[41], snapshot.path_feedforward_vx_mm_s);
  CommTuner_WriteFloat(&payload[45], snapshot.path_feedforward_vy_mm_s);
  payload[49] = snapshot.path_final_stage;

  frame_length = CommTuner_BuildFrame(COMM_TUNER_CMD_PATH_TELEMETRY,
                                      g_telemetry_sequence++, payload, sizeof(payload),
                                      g_path_telemetry_frame.data);
  g_path_telemetry_frame.length = frame_length;
  if (g_path_telemetry_pending != 0U)
  {
    ++g_telemetry_dropped_count;
  }
  g_path_telemetry_pending = 1U;
}

static void CommTuner_QueueTelemetry(void)
{
  AdvanceMotion_DebugSnapshot_t snapshot;
  uint8_t payload[COMM_TUNER_TELEMETRY_PAYLOAD_SIZE];
  uint16_t frame_length;
  uint32_t now_tick;

  if (AdvanceMotion_GetDebugSnapshot(&snapshot) != ADVANCE_MOTION_STATUS_OK)
  {
    return;
  }

  now_tick = HAL_GetTick();
  CommTuner_WriteU32(&payload[0], snapshot.tick);
  CommTuner_WriteU32(&payload[4], snapshot.pid_revision);
  CommTuner_WriteU32(&payload[8], g_telemetry_dropped_count);
  payload[12] = (uint8_t)snapshot.state;
  payload[13] = snapshot.flags;
  CommTuner_WriteU16(&payload[14], CommTuner_GetRemoteLinkStatus(now_tick));
  CommTuner_WriteFloat(&payload[16], snapshot.goal.x_mm);
  CommTuner_WriteFloat(&payload[20], snapshot.goal.y_mm);
  CommTuner_WriteFloat(&payload[24], snapshot.goal.yaw_deg);
  CommTuner_WriteFloat(&payload[28], snapshot.pose.x_mm);
  CommTuner_WriteFloat(&payload[32], snapshot.pose.y_mm);
  CommTuner_WriteFloat(&payload[36], snapshot.pose.yaw_deg);
  CommTuner_WriteFloat(&payload[40], snapshot.error_x_mm);
  CommTuner_WriteFloat(&payload[44], snapshot.error_y_mm);
  CommTuner_WriteFloat(&payload[48], snapshot.error_yaw_deg);
  CommTuner_WriteFloat(&payload[52], snapshot.command_vx_world_mm_s);
  CommTuner_WriteFloat(&payload[56], snapshot.command_vy_world_mm_s);
  CommTuner_WriteFloat(&payload[60], snapshot.command_wz_ccw_deg_s);
  CommTuner_WriteFloat(&payload[64], snapshot.measured_vx_world_mm_s);
  CommTuner_WriteFloat(&payload[68], snapshot.measured_vy_world_mm_s);
  CommTuner_WriteFloat(&payload[72], snapshot.measured_wz_deg_s);
  CommTuner_WriteFloat(&payload[76], snapshot.integral_x_mm_s);
  CommTuner_WriteFloat(&payload[80], snapshot.integral_y_mm_s);
  CommTuner_WriteFloat(&payload[84], snapshot.integral_yaw_deg_s);
  CommTuner_WriteFloat(&payload[88], snapshot.pose.wit_yaw_deg);
  CommTuner_WriteFloat(&payload[92], snapshot.pose.ops_yaw_deg);

  frame_length = CommTuner_BuildFrame(COMM_TUNER_CMD_TELEMETRY,
                                      g_telemetry_sequence++, payload, sizeof(payload),
                                      g_telemetry_frame.data);
  g_telemetry_frame.length = frame_length;
  if (g_telemetry_pending != 0U)
  {
    ++g_telemetry_dropped_count;
  }
  g_telemetry_pending = 1U;
}

static uint8_t CommTuner_MapMotionError(AdvanceMotion_Status_t status)
{
  switch (status)
  {
  case ADVANCE_MOTION_STATUS_BUSY:
    return COMM_TUNER_ERROR_BUSY;
  case ADVANCE_MOTION_STATUS_NO_ORIGIN:
    return COMM_TUNER_ERROR_NO_ORIGIN;
  case ADVANCE_MOTION_STATUS_NO_POSE:
    return COMM_TUNER_ERROR_NO_POSE;
  case ADVANCE_MOTION_STATUS_POSE_TIMEOUT:
    return COMM_TUNER_ERROR_POSE_TIMEOUT;
  default:
    return COMM_TUNER_ERROR_BAD_GOAL;
  }
}

static void CommTuner_HandleGoto(uint8_t sequence, const uint8_t *payload)
{
  WorldGoalPose2D_t goal;
  AdvanceMotion_Status_t status;

  goal.x_mm = CommTuner_ReadFloat(&payload[0]);
  goal.y_mm = CommTuner_ReadFloat(&payload[4]);
  goal.yaw_deg = CommTuner_ReadFloat(&payload[8]);
  goal.vmax_mm_s = CommTuner_ReadFloat(&payload[12]);
  goal.wmax_deg_s = CommTuner_ReadFloat(&payload[16]);
  goal.timeout_ms = CommTuner_ReadU32(&payload[20]);
  goal.goal_flags = payload[24];

  if (((goal.goal_flags & (ADVANCE_MOTION_GOAL_USE_POSITION | ADVANCE_MOTION_GOAL_USE_YAW)) == 0U) ||
      (((goal.goal_flags & ADVANCE_MOTION_GOAL_USE_POSITION) != 0U) &&
       ((goal.vmax_mm_s <= 0.0f) || (goal.vmax_mm_s > COMM_TUNER_GOTO_VMAX_MM_S))) ||
      (((goal.goal_flags & ADVANCE_MOTION_GOAL_USE_YAW) != 0U) &&
       ((goal.wmax_deg_s <= 0.0f) || (goal.wmax_deg_s > COMM_TUNER_GOTO_WMAX_DEG_S))) ||
      (goal.timeout_ms == 0U) || (goal.timeout_ms > COMM_TUNER_GOTO_TIMEOUT_MS))
  {
    CommTuner_SendError(COMM_TUNER_CMD_GOTO_POSE, sequence, COMM_TUNER_ERROR_BAD_GOAL, 1U);
    return;
  }

  status = AdvanceMotion_GotoPoseEx(&goal, CHASSIS_DEFAULT_ACC);
  if (status != ADVANCE_MOTION_STATUS_OK)
  {
    CommTuner_SendError(COMM_TUNER_CMD_GOTO_POSE, sequence,
                         CommTuner_MapMotionError(status), 1U);
    return;
  }

  g_remote_goal_active = 1U;
  g_remote_heartbeat_timeout = 0U;
  g_last_heartbeat_tick = HAL_GetTick();
  g_last_telemetry_tick = g_last_heartbeat_tick;
  CommTuner_SendAck(COMM_TUNER_CMD_GOTO_POSE, sequence, 0U, 0U);
}

static void CommTuner_HandlePathBegin(uint8_t sequence, const uint8_t *payload)
{
  uint16_t count = (uint16_t)payload[4] | ((uint16_t)payload[5] << 8U);

  if ((count < 2U) || (count > ADVANCE_MOTION_PATH_MAX_POINTS))
  {
    CommTuner_SendError(COMM_TUNER_CMD_PATH_BEGIN, sequence, COMM_TUNER_ERROR_BAD_GOAL, 1U);
    return;
  }
  CommTuner_ClearPathStaging();
  g_path_staging_id = CommTuner_ReadU32(&payload[0]);
  g_path_staging_count = count;
  g_path_staging_crc = (uint16_t)payload[6] | ((uint16_t)payload[7] << 8U);
  g_path_staging_state = COMM_TUNER_PATH_STAGING_RECEIVING;
  CommTuner_SendAck(COMM_TUNER_CMD_PATH_BEGIN, sequence, 0U, 0U);
}

static void CommTuner_HandlePathChunk(uint8_t sequence, const uint8_t *payload,
                                      uint16_t payload_length)
{
  uint32_t path_id = CommTuner_ReadU32(&payload[0]);
  uint16_t first = (uint16_t)payload[4] | ((uint16_t)payload[5] << 8U);
  uint8_t count = payload[6];
  uint16_t index;

  if ((g_path_staging_state != COMM_TUNER_PATH_STAGING_RECEIVING) ||
      (path_id != g_path_staging_id) || (count == 0U) ||
      (count > COMM_TUNER_PATH_CHUNK_MAX_POINTS) ||
      (first >= g_path_staging_count) ||
      (count > (uint8_t)(g_path_staging_count - first)) ||
      (payload_length != (uint16_t)(COMM_TUNER_PATH_CHUNK_HEADER_BYTES +
                                    ((uint16_t)count * COMM_TUNER_PATH_POINT_BYTES))))
  {
    CommTuner_SendError(COMM_TUNER_CMD_PATH_CHUNK, sequence, COMM_TUNER_ERROR_BAD_LENGTH, 1U);
    return;
  }
  for (index = 0U; index < count; ++index)
  {
    AdvanceMotion_PathPoint_t point;
    uint16_t point_index = (uint16_t)(first + index);

    if ((g_path_received[point_index / 8U] & (uint8_t)(1U << (point_index % 8U))) != 0U)
    {
      CommTuner_SendError(COMM_TUNER_CMD_PATH_CHUNK, sequence, COMM_TUNER_ERROR_BAD_GOAL, 1U);
      return;
    }
    point.x_mm = CommTuner_ReadFloat(&payload[COMM_TUNER_PATH_CHUNK_HEADER_BYTES + index * 12U]);
    point.y_mm = CommTuner_ReadFloat(&payload[COMM_TUNER_PATH_CHUNK_HEADER_BYTES + index * 12U + 4U]);
    point.yaw_deg = CommTuner_ReadFloat(&payload[COMM_TUNER_PATH_CHUNK_HEADER_BYTES + index * 12U + 8U]);
    if (CommTuner_IsPathPointValid(&point) == 0U)
    {
      CommTuner_SendError(COMM_TUNER_CMD_PATH_CHUNK, sequence, COMM_TUNER_ERROR_BAD_GOAL, 1U);
      return;
    }
    g_path_staging_points[point_index] = point;
    g_path_received[point_index / 8U] |= (uint8_t)(1U << (point_index % 8U));
    ++g_path_staging_received_count;
  }
  CommTuner_SendAck(COMM_TUNER_CMD_PATH_CHUNK, sequence, 0U, 0U);
}

static void CommTuner_HandlePathCommit(uint8_t sequence, const uint8_t *payload)
{
  AdvanceMotion_PathPoint_t *old_active;
  AdvanceMotion_RuntimeStatus_t motion_status;
  uint32_t path_id = CommTuner_ReadU32(payload);
  uint16_t index;
  const float minimum_segment_squared = ADVANCE_MOTION_PATH_MIN_SEGMENT_MM *
                                        ADVANCE_MOTION_PATH_MIN_SEGMENT_MM;

  (void)AdvanceMotion_GetStatus(&motion_status);
  if (motion_status.state == ADVANCE_MOTION_STATE_RUNNING)
  {
    CommTuner_SendError(COMM_TUNER_CMD_PATH_COMMIT, sequence, COMM_TUNER_ERROR_BUSY, 1U);
    return;
  }
  if ((g_path_staging_state != COMM_TUNER_PATH_STAGING_RECEIVING) ||
      (path_id != g_path_staging_id) ||
      (g_path_staging_received_count != g_path_staging_count) ||
      (CommTuner_Crc16((const uint8_t *)g_path_staging_points,
                        (uint16_t)(g_path_staging_count * COMM_TUNER_PATH_POINT_BYTES)) != g_path_staging_crc))
  {
    CommTuner_SendError(COMM_TUNER_CMD_PATH_COMMIT, sequence, COMM_TUNER_ERROR_BAD_GOAL, 1U);
    return;
  }
  for (index = 1U; index < g_path_staging_count; ++index)
  {
    float dx = g_path_staging_points[index].x_mm - g_path_staging_points[index - 1U].x_mm;
    float dy = g_path_staging_points[index].y_mm - g_path_staging_points[index - 1U].y_mm;

    if (((dx * dx) + (dy * dy)) < minimum_segment_squared)
    {
      CommTuner_SendError(COMM_TUNER_CMD_PATH_COMMIT, sequence, COMM_TUNER_ERROR_BAD_GOAL, 1U);
      return;
    }
  }
  old_active = g_path_active_points;
  g_path_active_points = g_path_staging_points;
  g_path_staging_points = old_active;
  g_path_active_id = g_path_staging_id;
  g_path_active_count = g_path_staging_count;
  CommTuner_ClearPathStaging();
  CommTuner_SendAck(COMM_TUNER_CMD_PATH_COMMIT, sequence, 0U, 0U);
}

static void CommTuner_HandlePathStart(uint8_t sequence, const uint8_t *payload)
{
  AdvanceMotion_Status_t status;

  if ((g_path_active_count < 2U) || (CommTuner_ReadU32(payload) != g_path_active_id))
  {
    CommTuner_SendError(COMM_TUNER_CMD_PATH_START, sequence, COMM_TUNER_ERROR_BAD_GOAL, 1U);
    return;
  }
  status = AdvanceMotion_FollowPathEx(g_path_active_points, g_path_active_count);
  if (status != ADVANCE_MOTION_STATUS_OK)
  {
    CommTuner_SendError(COMM_TUNER_CMD_PATH_START, sequence, CommTuner_MapMotionError(status), 1U);
    return;
  }
  g_remote_goal_active = 1U;
  g_remote_heartbeat_timeout = 0U;
  g_last_heartbeat_tick = HAL_GetTick();
  CommTuner_SendAck(COMM_TUNER_CMD_PATH_START, sequence, 0U, 0U);
}

static void CommTuner_HandleFrame(const uint8_t *frame, uint16_t frame_length)
{
  uint8_t command = frame[3];
  uint8_t sequence = frame[4];
  uint16_t payload_length = (uint16_t)frame[5] | ((uint16_t)frame[6] << 8U);
  const uint8_t *payload = &frame[7];
  uint16_t received_crc = (uint16_t)frame[frame_length - 2U] |
                          ((uint16_t)frame[frame_length - 1U] << 8U);

  if (CommTuner_Crc16(&frame[2], (uint16_t)(5U + payload_length)) != received_crc)
  {
    CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_CRC, 0U);
    return;
  }
  if (frame[2] != COMM_TUNER_PROTOCOL_VERSION)
  {
    CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_VERSION, 1U);
    return;
  }
  if ((g_response_cache_valid != 0U) &&
      (command == g_response_cache_command) &&
      (sequence == g_response_cache_sequence))
  {
    (void)CommTuner_QueueTransmit(g_response_cache.data, g_response_cache.length);
    return;
  }

  switch (command)
  {
  case COMM_TUNER_CMD_GET_PID:
    if (payload_length != 0U)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_LENGTH, 1U);
      return;
    }
    CommTuner_SendPid(command, sequence);
    break;

  case COMM_TUNER_CMD_SET_PID:
  {
    AdvanceMotion_PidConfig_t config;
    AdvanceMotion_Status_t status;
    uint32_t revision;

    if (payload_length != COMM_TUNER_SET_PID_PAYLOAD_SIZE)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_LENGTH, 1U);
      return;
    }
    config.kp_pos = CommTuner_ReadFloat(&payload[0]);
    config.ki_pos = CommTuner_ReadFloat(&payload[4]);
    config.kd_pos = CommTuner_ReadFloat(&payload[8]);
    config.kp_yaw = CommTuner_ReadFloat(&payload[12]);
    config.ki_yaw = CommTuner_ReadFloat(&payload[16]);
    config.kd_yaw = CommTuner_ReadFloat(&payload[20]);
    status = AdvanceMotion_RequestPidConfig(&config, &revision);
    if (status != ADVANCE_MOTION_STATUS_OK)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_PID, 1U);
      return;
    }
    CommTuner_SendAck(command, sequence, revision, 1U);
    break;
  }

  case COMM_TUNER_CMD_RESTORE_PID:
  {
    AdvanceMotion_Status_t status;
    uint32_t revision;

    if (payload_length != 0U)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_LENGTH, 1U);
      return;
    }
    status = AdvanceMotion_RestoreDefaultPid(&revision);
    if (status != ADVANCE_MOTION_STATUS_OK)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_PID, 1U);
      return;
    }
    CommTuner_SendAck(command, sequence, revision, 1U);
    break;
  }

  case COMM_TUNER_CMD_GOTO_POSE:
    if (payload_length != COMM_TUNER_GOTO_PAYLOAD_SIZE)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_LENGTH, 1U);
      return;
    }
    CommTuner_HandleGoto(sequence, payload);
    break;

  case COMM_TUNER_CMD_PATH_BEGIN:
    if (payload_length != 8U)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_LENGTH, 1U);
      return;
    }
    CommTuner_HandlePathBegin(sequence, payload);
    break;

  case COMM_TUNER_CMD_PATH_CHUNK:
    if (payload_length < COMM_TUNER_PATH_CHUNK_HEADER_BYTES)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_LENGTH, 1U);
      return;
    }
    CommTuner_HandlePathChunk(sequence, payload, payload_length);
    break;

  case COMM_TUNER_CMD_PATH_COMMIT:
    if (payload_length != 4U)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_LENGTH, 1U);
      return;
    }
    CommTuner_HandlePathCommit(sequence, payload);
    break;

  case COMM_TUNER_CMD_PATH_START:
    if (payload_length != 4U)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_LENGTH, 1U);
      return;
    }
    CommTuner_HandlePathStart(sequence, payload);
    break;

  case COMM_TUNER_CMD_PATH_ABORT:
    if (payload_length != 0U)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_LENGTH, 1U);
      return;
    }
    CommTuner_ClearPathStaging();
    CommTuner_SendAck(command, sequence, 0U, 0U);
    break;

  case COMM_TUNER_CMD_PATH_STATUS:
    if (payload_length != 0U)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_LENGTH, 1U);
      return;
    }
    CommTuner_SendPathStatus(command, sequence);
    break;

  case COMM_TUNER_CMD_GET_PATH_CONFIG:
    if (payload_length != 0U)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_LENGTH, 1U);
      return;
    }
    CommTuner_SendPathConfig(command, sequence);
    break;

  case COMM_TUNER_CMD_SET_PATH_CONFIG:
  {
    AdvanceMotion_PathControlConfig_t config;
    AdvanceMotion_Status_t status;
    uint32_t revision;
    float values[14U];
    uint8_t index;

    if (payload_length != COMM_TUNER_SET_PATH_CONFIG_PAYLOAD_SIZE)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_LENGTH, 1U);
      return;
    }
    for (index = 0U; index < 14U; ++index)
    {
      values[index] = CommTuner_ReadFloat(&payload[(uint16_t)index * 4U]);
    }
    config.kp_cross_track = values[0];
    config.kd_cross_track_velocity = values[1];
    config.kp_yaw = values[2];
    config.kd_yaw_rate = values[3];
    config.cruise_speed_mm_s = values[4];
    config.max_yaw_rate_deg_s = values[5];
    config.accel_mm_s2 = values[6];
    config.decel_mm_s2 = values[7];
    config.max_lateral_accel_mm_s2 = values[8];
    config.lookahead_min_mm = values[9];
    config.lookahead_base_mm = values[10];
    config.lookahead_speed_gain_s = values[11];
    config.lookahead_curve_gain_mm = values[12];
    config.lookahead_max_mm = values[13];
    status = AdvanceMotion_RequestPathControlConfig(&config, &revision);
    if (status != ADVANCE_MOTION_STATUS_OK)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_PATH_CONFIG, 1U);
      return;
    }
    CommTuner_SendAck(command, sequence, revision, 1U);
    break;
  }

  case COMM_TUNER_CMD_RESTORE_PATH_CONFIG:
  {
    AdvanceMotion_Status_t status;
    uint32_t revision;

    if (payload_length != 0U)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_LENGTH, 1U);
      return;
    }
    status = AdvanceMotion_RestoreDefaultPathControl(&revision);
    if (status != ADVANCE_MOTION_STATUS_OK)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_PATH_CONFIG, 1U);
      return;
    }
    CommTuner_SendAck(command, sequence, revision, 1U);
    break;
  }

  case COMM_TUNER_CMD_STOP:
    if (payload_length != 0U)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_LENGTH, 1U);
      return;
    }
    AdvanceMotion_Cancel();
    g_remote_goal_active = 0U;
    g_remote_heartbeat_timeout = 0U;
    CommTuner_ClearTelemetry();
    CommTuner_SendAck(command, sequence, 0U, 0U);
    break;

  case COMM_TUNER_CMD_HEARTBEAT:
    if (payload_length != 0U)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_LENGTH, 1U);
      return;
    }
    if (g_remote_goal_active != 0U)
    {
      g_last_heartbeat_tick = HAL_GetTick();
    }
    CommTuner_SendAck(command, sequence, 0U, 0U);
    break;

  case COMM_TUNER_CMD_SET_YAW_SOURCE:
    if (payload_length != 1U)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_LENGTH, 1U);
      return;
    }
    if ((payload[0] > (uint8_t)ADVANCE_WORLD_YAW_SOURCE_OPS) ||
        (AdvanceWorld_SetYawSource((AdvanceWorld_YawSource_t)payload[0]) != ADVANCE_WORLD_STATUS_OK))
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_GOAL, 1U);
      return;
    }
    AdvanceMotion_ResetYawControl();
    CommTuner_SendAck(command, sequence, 0U, 0U);
    break;

  case COMM_TUNER_CMD_RESET_ORIGIN:
  {
    AdvanceMotion_RuntimeStatus_t status;
    if (payload_length != 0U)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_LENGTH, 1U);
      return;
    }
    (void)AdvanceMotion_GetStatus(&status);
    if (status.state == ADVANCE_MOTION_STATE_RUNNING)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BUSY, 1U);
      return;
    }
    if (AdvanceWorld_ResetOrigin() != ADVANCE_WORLD_STATUS_OK)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_NO_POSE, 1U);
      return;
    }
    AdvanceMotion_ResetYawControl();
    CommTuner_SendAck(command, sequence, 0U, 0U);
    break;
  }

  case COMM_TUNER_CMD_GET_GOTO_STRATEGY:
    if (payload_length != 0U)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_LENGTH, 1U);
      return;
    }
    CommTuner_SendGotoStrategy(command, sequence);
    break;

  case COMM_TUNER_CMD_SET_GOTO_STRATEGY:
  {
    AdvanceMotion_Status_t status;

    if (payload_length != 1U)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_LENGTH, 1U);
      return;
    }
    status = AdvanceMotion_SetLargeYawAlignEnabled(payload[0]);
    if (status == ADVANCE_MOTION_STATUS_BUSY)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BUSY, 1U);
      return;
    }
    if (status != ADVANCE_MOTION_STATUS_OK)
    {
      CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_GOAL, 1U);
      return;
    }
    CommTuner_SendAck(command, sequence, 0U, 0U);
    break;
  }

  default:
    CommTuner_SendError(command, sequence, COMM_TUNER_ERROR_BAD_COMMAND, 1U);
    break;
  }
}

static void CommTuner_RemoveParsedBytes(uint16_t length)
{
  g_parse_size = (uint16_t)(g_parse_size - length);
  if (g_parse_size > 0U)
  {
    memmove(g_parse_buffer, &g_parse_buffer[length], g_parse_size);
  }
}

static void CommTuner_Parse(void)
{
  uint16_t payload_length;
  uint16_t frame_length;

  while (g_parse_size >= 2U)
  {
    if ((g_parse_buffer[0] != COMM_TUNER_SYNC0) ||
        (g_parse_buffer[1] != COMM_TUNER_SYNC1))
    {
      CommTuner_RemoveParsedBytes(1U);
      continue;
    }
    if (g_parse_size < 7U)
    {
      return;
    }
    payload_length = (uint16_t)g_parse_buffer[5] | ((uint16_t)g_parse_buffer[6] << 8U);
    if (payload_length > COMM_TUNER_MAX_PAYLOAD_SIZE)
    {
      CommTuner_SendError(g_parse_buffer[3], g_parse_buffer[4], COMM_TUNER_ERROR_BAD_LENGTH, 0U);
      CommTuner_RemoveParsedBytes(1U);
      continue;
    }
    frame_length = (uint16_t)(COMM_TUNER_FRAME_OVERHEAD + payload_length);
    if (g_parse_size < frame_length)
    {
      return;
    }
    CommTuner_HandleFrame(g_parse_buffer, frame_length);
    CommTuner_RemoveParsedBytes(frame_length);
  }
}

static void CommTuner_ParseBytes(const uint8_t *data, uint16_t length)
{
  uint16_t index;

  for (index = 0U; index < length; ++index)
  {
    if (g_parse_size >= COMM_TUNER_FRAME_MAX_SIZE)
    {
      CommTuner_RemoveParsedBytes(1U);
    }
    g_parse_buffer[g_parse_size++] = data[index];
    CommTuner_Parse();
  }
}

static void CommTuner_StartTransmit(void)
{
  CommTuner_TxFrame_t *frame;
  uint16_t frame_length;
  uint32_t primask;
  uint8_t telemetry_selected = 0U;

  if ((g_uart == NULL) || (g_tx_busy != 0U) ||
      ((g_tx_queue_count == 0U) && (g_telemetry_pending == 0U) &&
       (g_path_telemetry_pending == 0U)))
  {
    return;
  }

  if (g_tx_queue_count > 0U)
  {
    frame = &g_tx_queue[g_tx_queue_tail];
    g_tx_queue_tail = CommTuner_NextQueueIndex(g_tx_queue_tail);
    --g_tx_queue_count;
  }
  else
  {
    primask = __get_PRIMASK();
    __disable_irq();
    if ((g_telemetry_pending == 0U) && (g_path_telemetry_pending == 0U))
    {
      if (primask == 0U)
      {
        __enable_irq();
      }
      return;
    }
    if (g_path_telemetry_pending != 0U)
    {
      frame = &g_path_telemetry_frame;
      g_path_telemetry_pending = 0U;
    }
    else
    {
      frame = &g_telemetry_frame;
      g_telemetry_pending = 0U;
    }
    frame_length = frame->length;
    memcpy(g_tx_dma, frame->data, frame_length);
    if (primask == 0U)
    {
      __enable_irq();
    }
    telemetry_selected = 1U;
  }
  if (telemetry_selected == 0U)
  {
    frame_length = frame->length;
    memcpy(g_tx_dma, frame->data, frame_length);
  }
  g_tx_busy = 1U;
  if (HAL_UART_Transmit_DMA(g_uart, g_tx_dma, frame_length) != HAL_OK)
  {
    g_tx_busy = 0U;
    if (telemetry_selected != 0U)
    {
      ++g_telemetry_dropped_count;
    }
    else
    {
      ++g_rx_dropped_count;
    }
    return;
  }
}

static void CommTuner_QueueReceivedBytes(uint16_t current_pos)
{
  CommTuner_RxMessage_t *message;
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
  g_rx_dropped_count = 0U;
  g_response_cache_valid = 0U;
  g_telemetry_sequence = 0U;
  g_telemetry_dropped_count = 0U;
  g_remote_goal_active = 0U;
  g_remote_heartbeat_timeout = 0U;
  g_last_heartbeat_tick = 0U;
  g_last_telemetry_tick = 0U;
  g_last_path_telemetry_tick = 0U;
  g_path_active_points = g_path_buffer_a;
  g_path_staging_points = g_path_buffer_b;
  g_path_active_id = 0U;
  g_path_active_count = 0U;
  CommTuner_ClearPathStaging();
  CommTuner_ClearReceiveState();
  CommTuner_ClearTransmitState();
  return CommTuner_StartRx();
}

void CommTuner_Process(void)
{
  CommTuner_RxMessage_t *message;

  while (g_rx_queue_count > 0U)
  {
    message = &g_rx_queue[g_rx_queue_tail];
    CommTuner_ParseBytes(message->data, message->length);
    g_rx_queue_tail = CommTuner_NextQueueIndex(g_rx_queue_tail);
    --g_rx_queue_count;
  }
  CommTuner_StartTransmit();
}

void CommTuner_Update(void)
{
  AdvanceMotion_RuntimeStatus_t status;
  uint32_t now_tick;

  now_tick = HAL_GetTick();
  if ((now_tick - g_last_telemetry_tick) >= COMM_TUNER_TELEMETRY_PERIOD_MS)
  {
    g_last_telemetry_tick = now_tick;
    CommTuner_QueueTelemetry();
  }
  if ((now_tick - g_last_path_telemetry_tick) >= COMM_TUNER_PATH_TELEMETRY_PERIOD_MS)
  {
    g_last_path_telemetry_tick = now_tick;
    CommTuner_QueuePathTelemetry();
  }

  /* Telemetry is available in every motion state; only motion safety needs an active goal. */
  if (g_remote_goal_active == 0U)
  {
    return;
  }
  if ((AdvanceMotion_GetStatus(&status) != ADVANCE_MOTION_STATUS_OK) ||
      (status.state != ADVANCE_MOTION_STATE_RUNNING))
  {
    g_remote_goal_active = 0U;
    return;
  }
  if ((now_tick - g_last_heartbeat_tick) >= COMM_TUNER_HEARTBEAT_TIMEOUT_MS)
  {
    AdvanceMotion_Cancel();
    g_remote_goal_active = 0U;
    g_remote_heartbeat_timeout = 1U;
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
    CommTuner_ClearReceiveState();
    CommTuner_ClearTransmitState();
    (void)CommTuner_StartRx();
  }
}

uint32_t CommTuner_GetDroppedCount(void)
{
  return g_rx_dropped_count;
}
