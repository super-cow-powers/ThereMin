/**
  ******************************************************************************
  * @file    stm32l1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "main.h"
#include "stm32l1xx_it.h"
#include "stm32l1xx.h"
#include <stdint.h>

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


/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
}

