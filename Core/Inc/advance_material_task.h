#ifndef __ADVANCE_MATERIAL_TASK_H__
#define __ADVANCE_MATERIAL_TASK_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#include "comm_jetson.h"

#define ADVANCE_MATERIAL_TASK_ITEM_COUNT ((uint8_t)3U)

typedef struct
{
  uint8_t color1[ADVANCE_MATERIAL_TASK_ITEM_COUNT];    /*!< 第1轮物料颜色顺序，数值范围 1~4。 */
  uint8_t position1[ADVANCE_MATERIAL_TASK_ITEM_COUNT]; /*!< 第1轮粗加工区和暂存区底层位置，数值范围 1~3。 */
  uint8_t color2[ADVANCE_MATERIAL_TASK_ITEM_COUNT];    /*!< 第2轮物料颜色顺序，数值范围 1~4。 */
  uint8_t position2[ADVANCE_MATERIAL_TASK_ITEM_COUNT]; /*!< 第2轮粗加工区位置，数值范围 1~3。 */
} AdvanceMaterialTask_t;

/** @brief 将固定 15 字节二维码任务码解析为任务对象。 */
bool AdvanceMaterialTask_ParseCode(
    const char code[DETECT_QR_CODE_LENGTH + 1U],
    AdvanceMaterialTask_t *task);

/** @brief 第1轮：对准原料转盘中心后，在固定取料位按颜色顺序依次抓取到车载托盘。 */
void AdvanceMaterialTask_Collect1(const AdvanceMaterialTask_t *task);

/** @brief 第1轮：按位置依次放入粗加工区，再依次抓回原车载托盘槽位。 */
void AdvanceMaterialTask_Process1(const AdvanceMaterialTask_t *task);

/** @brief 第1轮：按位置依次放入暂存区底层。 */
void AdvanceMaterialTask_Store1(const AdvanceMaterialTask_t *task);

/** @brief 第2轮：对准原料转盘中心后，在固定取料位按颜色顺序依次抓取到车载托盘。 */
void AdvanceMaterialTask_Collect2(const AdvanceMaterialTask_t *task);

/** @brief 第2轮：按位置依次放入粗加工区，再依次抓回原车载托盘槽位。 */
void AdvanceMaterialTask_Process2(const AdvanceMaterialTask_t *task);

/** @brief 第2轮：按同色关系依次堆叠到第1轮物料上方。 */
void AdvanceMaterialTask_Stack2(const AdvanceMaterialTask_t *task);

#ifdef __cplusplus
}
#endif

#endif
