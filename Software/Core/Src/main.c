#include "main.h"
#include "stm32l152xe.h"
#include "stm32l1xx_it.h"
#include "sys-setup.h"
#include "waveform.h"
#include <stdint.h>

// DAC on PA4
static volatile TIM_TypeDef *tim = TIM7;
static volatile DAC_TypeDef *dac = DAC;
static volatile DMA_Channel_TypeDef* dacDma = DMA1_Channel3;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  
  platform_init();
  start_output();
  NVIC_EnableIRQ(TIM7_IRQn);
  while (1) {
    
  }
}


void TIM7_IRQHandler() {
  __asm("BKPT");
  if (TIM7->SR) {
    TIM7->SR = 0;
    NVIC_ClearPendingIRQ(TIM7_IRQn);
  }
}
