#include "stdint.h"

void SysTick_Handler(void);
void HAL_InitTick(uint32_t TickPriority);
uint32_t HAL_GetTick(void);
