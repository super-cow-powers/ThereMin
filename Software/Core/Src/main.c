#include "main.h"
#include "stm32l152xe.h"
#include "stm32l1xx_it.h"
#include "sys-setup.h"
#include "waveform.h"
#include <stdint.h>

volatile uint32_t SYSTICK_VAL = 0;

// DAC on PA4
static volatile TIM_TypeDef *tim = TIM7;
static volatile TIM_TypeDef *tim2 = TIM2;
static volatile TIM_TypeDef *tim3 = TIM3;

static volatile DAC_TypeDef *dac = DAC;
static volatile DMA_Channel_TypeDef* dacDma = DMA1_Channel3;

volatile uint16_t volumeInputBuffer[FRQ_CNT_SAMPLES];
volatile uint16_t pitchInputBuffer[FRQ_CNT_SAMPLES];
volatile uint16_t volumeInputMean;
volatile uint16_t pitchInputMean;

volatile uint16_t volumeCal;
volatile uint16_t pitchCal;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  
  platform_init();
  start_output();
  //NVIC_EnableIRQ(TIM7_IRQn);
  //NVIC_EnableIRQ(TIM2_IRQn);
  NVIC_EnableIRQ(TIM3_IRQn);
  NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  for (uint32_t i = 0; i<3*UINT16_MAX; i++) {

  }
  volumeCal = volumeInputMean;
  pitchCal = pitchInputMean;
  int32_t volumeScale;
  int32_t pitchScale;
  while (1) {
    //Try leaving pitch-scaling as-is?
    pitchScale = pitchInputMean - pitchCal;
    
    
    volumeScale = volumeCal - volumeInputMean;
    //Calculate scaled number by multiplying 
  }
}

void DMA1_Channel2_IRQHandler () {
  uint32_t count = 0;
  if (DMA1->ISR & 1U << DMA_ISR_TCIF2_Pos) {
    DMA1->IFCR |= 1U << DMA_IFCR_CTCIF2_Pos;
    for (uint8_t i = 0; i<FRQ_CNT_SAMPLES; i++) {
      count += pitchInputBuffer[i];
    }
    pitchInputMean = count >> 3; //Divide by 8
  }
}

void DMA1_Channel6_IRQHandler () {
  uint32_t count = 0;
  if (DMA1->ISR & 1U << DMA_ISR_TCIF6_Pos) {
    DMA1->IFCR |= 1U << DMA_IFCR_CTCIF6_Pos;
    for (uint8_t i = 0; i<FRQ_CNT_SAMPLES; i++) {
      count += volumeInputBuffer[i];
    }
    volumeInputMean = count >> 3; //Divide by 8
  }
}

void TIM2_IRQHandler() {
  //__asm("BKPT");
  if (TIM2->SR) {
    TIM2->SR = 0;
    NVIC_ClearPendingIRQ(TIM2_IRQn);
  }
}

void TIM3_IRQHandler() {
  if (TIM3->CCR1) {
    //__asm("BKPT");
  }
  if (TIM3->SR) {
    TIM3->SR = 0;
    NVIC_ClearPendingIRQ(TIM3_IRQn);
  }
}

void TIM7_IRQHandler() {
  //__asm("BKPT");
  if (TIM7->SR) {
    TIM7->SR = 0;
    NVIC_ClearPendingIRQ(TIM7_IRQn);
  }
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  SYSTICK_VAL ++;
}

