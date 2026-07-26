#ifndef COMM_JETSON_H
#define COMM_JETSON_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"

/** Jetson 视觉检测通信模块的默认检测周期，单位为 ms。 */
#define DETECT_DEFAULT_PERIOD_MS ((uint16_t)40U)
/** 单次检测结果允许保存的最大目标数量。 */
#define DETECT_TARGET_MAX ((uint8_t)8U)
/** 二维码任务码的固定 ASCII 字节长度。 */
#define DETECT_QR_CODE_LENGTH ((uint8_t)15U)
/** 二维码阻塞读取的最长等待时间，单位为 ms。 */
#define DETECT_QR_TIMEOUT_MS ((uint32_t)2000U)

/** 检测通信及检测任务的返回状态。 */
typedef enum
{
  DETECT_STATUS_OK = 0,       /**< 操作成功。 */
  DETECT_STATUS_NO_TARGET,    /**< 当前没有有效检测目标。 */
  DETECT_STATUS_BAD_COMMAND,  /**< 收到不支持的命令。 */
  DETECT_STATUS_BAD_LENGTH,   /**< 收到的数据长度不正确。 */
  DETECT_STATUS_BAD_PERIOD,   /**< 检测周期参数不合法。 */
  DETECT_STATUS_UART_ERROR,   /**< UART 通信发生错误。 */
  DETECT_STATUS_BUSY,         /**< UART DMA 发送缓冲区正在使用。 */
  DETECT_STATUS_TIMEOUT,      /**< 等待二维码结果超时。 */
  DETECT_STATUS_BAD_PARAMETER /**< 调用参数不合法。 */
} Detect_Status_t;

/** 单个视觉检测目标的结果。 */
typedef struct
{
  uint8_t type;          /**< 目标类型，由 Jetson 协议定义。 */
  int16_t x;             /**< 目标中心的 X 坐标。 */
  int16_t y;             /**< 目标中心的 Y 坐标。 */
  uint8_t confidence;    /**< 检测置信度。 */
  uint8_t measured;      /**< 目标测量结果是否有效。 */
  uint8_t support_count; /**< 支持该目标的检测帧数量。 */
} Detect_Target_t;

/** 一次检测返回的目标列表。 */
typedef struct
{
  uint8_t count; /**< 当前列表中的有效目标数量。 */
  Detect_Target_t targets[DETECT_TARGET_MAX]; /**< 目标结果数组。 */
} Detect_TargetList_t;

/** 圆盘中心检测结果。 */
typedef struct
{
  Detect_Status_t status; /**< 中心检测状态。 */
  int16_t x;              /**< 圆盘中心的 X 坐标。 */
  int16_t y;              /**< 圆盘中心的 Y 坐标。 */
  uint8_t support_count;  /**< 支持该中心结果的检测帧数量。 */
  uint8_t measured_count; /**< 已参与测量的有效样本数量。 */
} Detect_DiskCenter_t;

/** 启动颜色目标检测。 */
Detect_Status_t detect_color_start(void);
/** 启动圆形目标检测。 */
Detect_Status_t detect_circle_start(void);
/** 启动圆盘中心检测。 */
Detect_Status_t detect_disk_center_start(void);
/** 启动一次二维码检测会话。 */
Detect_Status_t detect_qr_start(void);
/** 停止当前检测任务。 */
Detect_Status_t detect_stop(void);

/** 获取最近一次目标检测结果。 */
uint8_t detect_get_targets(Detect_TargetList_t *result);
/** 获取最近一次圆盘中心检测结果。 */
uint8_t detect_get_disk_center(Detect_DiskCenter_t *result);
/** 获取最近一次二维码任务码，返回值为 1 表示成功复制新结果。 */
uint8_t detect_get_qr(char code[DETECT_QR_CODE_LENGTH + 1U]);
/** 启动二维码检测并阻塞等待一次结果或超时。 */
Detect_Status_t detect_qr_read_blocking(char code[DETECT_QR_CODE_LENGTH + 1U]);
/** 查询检测任务是否处于运行状态。 */
uint8_t detect_is_active(void);
/** 查询最近一次检测结果是否在指定超时时间内刷新。 */
uint8_t detect_is_fresh(uint32_t timeout_ms);

/** 初始化 Jetson 通信模块并绑定 UART。 */
void CommJetson_Init(UART_HandleTypeDef *huart);
/** @brief 由 TIM6 每 1 ms 调用，推进二维码等待和通信超时状态。 */
void CommJetson_Update(void);
/** 处理 UART 接收事件，并解析收到的数据。 */
void CommJetson_OnUartRxEvent(UART_HandleTypeDef *huart, uint16_t size);
/** 处理 UART 错误事件并恢复接收。 */
void CommJetson_OnUartError(UART_HandleTypeDef *huart);
/** 处理 UART DMA 发送完成事件。 */
void CommJetson_OnUartTxComplete(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
