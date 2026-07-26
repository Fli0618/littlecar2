#ifndef ADVANCE_CONTROL_H
#define ADVANCE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  ADVANCE_CONTROL_NONE = 0U,
  ADVANCE_CONTROL_WORLD,
  ADVANCE_CONTROL_VISUAL
} AdvanceControl_Mode_t;

void AdvanceControl_Init(void);
void AdvanceControl_SetMode(AdvanceControl_Mode_t mode);
AdvanceControl_Mode_t AdvanceControl_GetMode(void);

#ifdef __cplusplus
}
#endif

#endif
