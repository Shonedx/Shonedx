#include "ticks.h"

#include "stm32f4xx.h"

static __IO uint32_t uwTick;

void SysTick_Handler(void)
{
  uwTick++;
}

void HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClocksTypeDef RCC_Clocks;
  uint32_t uwTimclock = 0;
  uint32_t PrescalerValue = 0;


  RCC_GetClocksFreq(&RCC_Clocks);

  uwTimclock = RCC_Clocks.HCLK_Frequency;

  PrescalerValue = (uint32_t)(uwTimclock / 1000000);


  SysTick_Config(PrescalerValue);

  NVIC_SetPriority(SysTick_IRQn, TickPriority);
}

uint32_t HAL_GetTick(void)
{
  return uwTick;
}






/****************************************************/













