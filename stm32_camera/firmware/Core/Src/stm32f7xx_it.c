/* Interrupt handlers. Anything not listed here falls into the startup file's Default_Handler. */
#include "main.h"

void SysTick_Handler(void)
{
  HAL_IncTick();
}

/* Camera capture (capture.c) */
extern DCMI_HandleTypeDef hdcmi;

void DCMI_IRQHandler(void)
{
  HAL_DCMI_IRQHandler(&hdcmi);
}

void DMA2_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hdcmi.DMA_Handle);
}

void HardFault_Handler(void)
{
  Error_Handler();
}

void MemManage_Handler(void)
{
  Error_Handler();
}

void BusFault_Handler(void)
{
  Error_Handler();
}

void UsageFault_Handler(void)
{
  Error_Handler();
}
