
#include "stm32l152xe.h"
#include "stm32l1xx.h"
#include "system_stm32l1xx.h"
#include <stdint.h>

/** @brief Configures the clock to use HSE (external oscillator) and the PLL
   to get SysClk == AHB == APB1 == APB2 == 24MHz */
static void
_configClock (void)
{
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable SYSCFG clock
  RCC->APB1ENR |= RCC_APB1ENR_PWREN; // Enable PWR config clock

  FLASH->ACR |= FLASH_ACR_ACC64;
  while ((FLASH->ACR & FLASH_ACR_ACC64_Msk) != FLASH_ACR_ACC64_Msk);
  FLASH->ACR |= FLASH_ACR_LATENCY_Msk; // Set 1 wait state on NVM
  while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != FLASH_ACR_LATENCY_Msk);
  
  RCC->CR &= ~((uint32_t)1 << 24); //Disable PLL
  while (RCC->CR & ((uint32_t)1 << 25)); //Wait till PLL is off
  RCC->CR |= (uint32_t)1; //Enable HSI16
  while (!(RCC->CR & (uint32_t)1 << 1)); //Wait till HSI16 is ready
  
  //Set PLL for 24MHz output ((16/4)*6 = 24)
  RCC->CFGR |= (uint32_t)0b11 << 22; //Divide HSI16 by 4
  RCC->CFGR |= (uint32_t)0b0011 << 18; //Multiply by 8
  RCC->CFGR &= ~RCC_CFGR_PLLSRC_Msk;
  
  RCC->CR |= (uint32_t)1 << 24; //Enable PLL
  while ((RCC->CR & ((uint32_t)1 << 25)) != ((uint32_t)1 << 25)); //Wait till PLL is stable
  RCC->CFGR |= (uint32_t)0b11; //Set PLL as system clock
  while ((RCC->CFGR & ((uint32_t)0b11 << 2)) != ((uint32_t)0b11 << 2)); //Wait till PLL is set as system clock
  
  SystemCoreClockUpdate();
  
  
}

static void ioInit (void) {
  GPIOA->MODER |= 0b01 << 8;
  
}

/** @brief Configures the NVIC with the used IRQs.
 */
static void
_configIRQs (void)
{
  
}

void
enable_tick ()
{
  SystemCoreClockUpdate ();
  SysTick_Config (SystemCoreClock / 1000);
  NVIC_EnableIRQ (SysTick_IRQn);
  NVIC_SetPriority (SysTick_IRQn,
                    0); // Set Systick to highest priority
}

void
platform_init ()
{
  SystemInit();
  _configClock ();
  _configIRQs ();
  enable_tick();
  ioInit();
}

void
software_reset()
{
  NVIC_SystemReset();
}
