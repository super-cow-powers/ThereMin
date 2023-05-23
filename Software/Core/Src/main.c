#include "main.h"
#include "stm32l152xe.h"
#include "stm32l1xx_it.h"
#include "sys-setup.h"
#include <stdint.h>

// DAC on PA4

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  uint8_t val = 0;
  platform_init();
  while (1) {
    if (SYSTICK_VAL % 100) {
      val ^=1;
      GPIOA->ODR = val;
    }
  }
}
