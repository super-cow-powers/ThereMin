
#include "nucleo-l152_defs.h"
#include "stm32l152xe.h"
#include "stm32l1xx.h"
#include "system_stm32l1xx.h"
#include "sys-setup.h"
#include "waveform.h"
#include <string.h>
#include <stdint.h>


/** @brief Configures the clock to use HSE (external oscillator) and the PLL
   to get SysClk == AHB == APB1 == APB2 == 24MHz */
static void
_configClock (void)
{
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable SYSCFG clock
  RCC->APB1ENR |= RCC_APB1ENR_PWREN; // Enable PWR config clock

  uint32_t pwrVal = PWR->CR;
  pwrVal &= ~(0b11 << PWR_CR_VOS_Pos);
  pwrVal |= 0b01 << PWR_CR_VOS_Pos;
  PWR->CR = pwrVal;
  
  FLASH->ACR |= FLASH_ACR_ACC64;
  while ((FLASH->ACR & FLASH_ACR_ACC64_Msk) != FLASH_ACR_ACC64_Msk);
  FLASH->ACR |= FLASH_ACR_LATENCY_Msk; // Set 1 wait state on NVM
  while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != FLASH_ACR_LATENCY_Msk);
  
  RCC->CR &= ~((uint32_t)1 << 24); //Disable PLL
  while (RCC->CR & ((uint32_t)1 << 25)); //Wait till PLL is off
  RCC->CR |= (uint32_t)1; //Enable HSI16
  while (!(RCC->CR & (uint32_t)1 << 1)); //Wait till HSI16 is ready
  
  //Set PLL for 32MHz output ((16/4)*8 = 32)
  RCC->CFGR |= (uint32_t)0b11 << 22; //Divide HSI16 by 4
  RCC->CFGR |= (uint32_t)0b0011 << 18; //Multiply by 8
  RCC->CFGR &= ~RCC_CFGR_PLLSRC_Msk;
  
  RCC->CR |= (uint32_t)1 << 24; //Enable PLL
  while ((RCC->CR & ((uint32_t)1 << 25)) != ((uint32_t)1 << 25)); //Wait till PLL is stable
  RCC->CFGR |= (uint32_t)0b11; //Set PLL as system clock
  while ((RCC->CFGR & ((uint32_t)0b11 << 2)) != ((uint32_t)0b11 << 2)); //Wait till PLL is set as system clock
  
  SystemCoreClockUpdate();
}

static void _configureDacDMA(void) {
  DMA1_Channel3->CCR |= 0b11 << DMA_CCR_PL_Pos; //Max Priority
  DMA1_Channel3->CCR |= 0b01 << DMA_CCR_MSIZE_Pos; //16 Bit data (memory)
  DMA1_Channel3->CCR |= 0b01 << DMA_CCR_PSIZE_Pos; //16 Bit data (peripheral)
  DMA1_Channel3->CCR |= 0b1 << DMA_CCR_MINC_Pos; //Auto-increment memory position
  DMA1_Channel3->CCR |= 0b1 << DMA_CCR_CIRC_Pos; //Circular buffer mode
  DMA1_Channel3->CCR |= 0b1 << DMA_CCR_DIR_Pos; //Read from memory to peripheral
  DMA1_Channel3->CNDTR = SAMPLES_NUMBER; //Set length of data buffer
  DMA1_Channel3->CPAR = (uint32_t)(&(DAC->DHR12R1)); //Data to DAC output
  DMA1_Channel3->CMAR = (uint32_t)((uint16_t*)ACTIVE_WF_BUF); //Data from Waveform Buffer
  DMA1_Channel3->CCR |= 0b1 << DMA_CCR_EN_Pos; //Enable Channel
  DMA1_Channel3->CCR |= 0b1 << DMA_CCR_TCIE_Pos; //Transfer Complete IRQ Enable
}

static void _configureTim3DMA(void) {
  //Volume Timer DMA
  DMA1_Channel6->CCR |= 0b01 << DMA_CCR_PL_Pos; //Medium Priority
  DMA1_Channel6->CCR |= 0b01 << DMA_CCR_MSIZE_Pos; //16 Bit data (memory)
  DMA1_Channel6->CCR |= 0b01 << DMA_CCR_PSIZE_Pos; //16 Bit data (peripheral)
  DMA1_Channel6->CCR |= 0b1 << DMA_CCR_MINC_Pos; //Auto-increment memory position
  DMA1_Channel6->CCR |= 0b1 << DMA_CCR_CIRC_Pos; //Circular buffer mode
  DMA1_Channel6->CCR |= 0b1 << DMA_CCR_TCIE_Pos; //Transfer Complete IRQ Enable
  DMA1_Channel6->CCR &= ~(0b1 << DMA_CCR_DIR_Pos); //Read from peripheral to memory
  DMA1_Channel6->CNDTR = VOL_CNT_SAMPLES; //Set length of data buffer
  DMA1_Channel6->CPAR = (uint32_t)(&(TIM3->CCR1)); //Data from Timer CCR1 output
  DMA1_Channel6->CMAR = (uint32_t)((uint16_t*)volumeInputBuffer); //Data to frequency Buffer
  DMA1_Channel6->CCR |= 0b1 << DMA_CCR_EN_Pos; //Enable Channel
}

static void _configureTim2DMA(void) {
  //Pitch Timer DMA
  DMA1_Channel2->CCR |= 0b01 << DMA_CCR_PL_Pos; //Medium Priority
  DMA1_Channel2->CCR |= 0b01 << DMA_CCR_MSIZE_Pos; //16 Bit data (memory)
  DMA1_Channel2->CCR |= 0b01 << DMA_CCR_PSIZE_Pos; //16 Bit data (peripheral)
  DMA1_Channel2->CCR |= 0b1 << DMA_CCR_MINC_Pos; //Auto-increment memory position
  DMA1_Channel2->CCR |= 0b1 << DMA_CCR_CIRC_Pos; //Circular buffer mode
  DMA1_Channel2->CCR |= 0b1 << DMA_CCR_TCIE_Pos; //Transfer Complete IRQ Enable
  DMA1_Channel2->CCR &= ~(0b1 << DMA_CCR_DIR_Pos); //Read from peripheral to memory
  DMA1_Channel2->CNDTR = FRQ_CNT_SAMPLES; //Set length of data buffer
  DMA1_Channel2->CPAR = (uint32_t)(&(TIM2->CCR1)); //Data from Timer CCR1 output
  DMA1_Channel2->CMAR = (uint32_t)((uint16_t*)pitchInputBuffer); //Data to frequency Buffer
  DMA1_Channel2->CCR |= 0b1 << DMA_CCR_EN_Pos; //Enable Channel
}

/** Configure the DMA Engine */
static void _configureDMA(void) {
  RCC->AHBRSTR |= RCC_AHBRSTR_DMA1RST;
  RCC->AHBRSTR &= ~RCC_AHBRSTR_DMA1RST;
  RCC->AHBENR |= RCC_AHBENR_DMA1EN; //Using DMA1
  
  _configureDacDMA();
  _configureTim3DMA();
  _configureTim2DMA();
}

static void _configureDacTimer(void) {
  //Timer 7
  RCC->APB1RSTR |= (RCC_APB1RSTR_TIM7RST);
  RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM7RST);
  RCC->APB1ENR |= (RCC_APB1ENR_TIM7EN);
  TIM7->PSC = 0; //32MHz/(31+1) for 1MHz input gives 15Hz minimum output
  TIM7->CR1 &= ~( TIM_CR1_OPM | TIM_CR1_UDIS); //Auto-reload preload, multi-pulse mode, enable Update Event
  TIM7->CR1 |= 0b1 << TIM_CR1_URS;
  TIM7->DIER |= TIM_DIER_UDE_Msk; //Enable DMA Request on update
  TIM7->ARR = INITIAL_DAC_ARR;
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM7_STOP; //Freeze on breakpoint
}

static void _configurePitchTimer(void) {
  //Timer 2
  RCC->APB1RSTR |= (RCC_APB1RSTR_TIM2RST);
  RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM2RST);
  RCC->APB1ENR |= (RCC_APB1ENR_TIM2EN);
  TIM2->CR2 &= ~(0b1U << TIM_CR2_CCDS_Pos); //DMA on capture compare
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM2_STOP; //Freeze on breakpoint

  TIM2->CCMR1 |= 0b10 << TIM_CCMR1_CC1S_Pos; //Input 2 to CC1
  TIM2->CCMR1 |= 0b0011 << TIM_CCMR1_IC1F_Pos; //8-cycle Filter
  TIM2->SMCR |= 0b110 << TIM_SMCR_TS_Pos; //Timer Input 2 as Trigger
  TIM2->SMCR |= 0b100U << TIM_SMCR_SMS_Pos; //Slave Reset Mode
  TIM2->CCER &= ~((0b1 << TIM_CCER_CC1P_Pos) | (0b1 << TIM_CCER_CC1NP_Pos)); //Rising Edge
  TIM2->CCMR1 &= ~(0b11 << TIM_CCMR1_IC1PSC_Pos); //No Prescaler
  TIM2->DIER |= 1U << TIM_DIER_UDE_Pos; //Enable DMA Request on Update. Get data out when registers update.
  TIM2->CCER |= TIM_CCER_CC1E; //Enable CC1
  TIM2->CR1 |= 1U; //Enable Timer
}

static void _configureVolumeTimer(void) {
  //Timer 3
  
  RCC->APB1RSTR |= (RCC_APB1RSTR_TIM3RST);
  RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM3RST);
  RCC->APB1ENR |= (RCC_APB1ENR_TIM3EN);
  TIM3->CR2 &= ~(0b1U << TIM_CR2_CCDS_Pos); //DMA on capture compare
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM3_STOP; //Freeze on breakpoint

  TIM3->CCMR1 |= 0b01 << TIM_CCMR1_CC1S_Pos; //Input 1 to CC1
  TIM3->CCMR1 |= 0b0011 << TIM_CCMR1_IC1F_Pos; //8-cycle Filter
  TIM3->SMCR |= 0b101 << TIM_SMCR_TS_Pos; //Timer Input 1 as Trigger
  TIM3->SMCR |= 0b100U << TIM_SMCR_SMS_Pos; //Slave Reset Mode
  TIM3->CCER &= ~((0b1 << TIM_CCER_CC1P_Pos) | (0b1 << TIM_CCER_CC1NP_Pos));
  TIM3->CCMR1 &= ~(0b11 << TIM_CCMR1_IC1PSC_Pos); //No Prescaler
  TIM3->DIER |= 1U << TIM_DIER_CC1DE_Pos; //Enable DMA Request on CC1. Get data out when CC1 updates.
  TIM3->CCER |= TIM_CCER_CC1E;
  TIM3->CR1 |= 1U;
}

void configure_timers() {
  _configureDacTimer();
  _configurePitchTimer();
  _configureVolumeTimer();
}

void configure_dac() {
  RCC->APB1RSTR |= RCC_APB1RSTR_DACRST;
  RCC->APB1RSTR &= ~RCC_APB1RSTR_DACRST;
  RCC->APB1ENR |= RCC_APB1ENR_DACEN;
  DAC->DHR12R1 = 2000;
  DAC->CR |=   DAC_CR_BOFF1 |DAC_CR_EN1; //No output buffer, enable DAC
}

void start_output() {
  memcpy((uint16_t*)ACTIVE_WF_BUF, SINE_WF, SAMPLES_NUMBER*sizeof(uint16_t)); //Place Sine into output buffer
  TIM7->CR1 |= 0b1;
}

void configure_gpio (void) {
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN_Msk; //Enable clocks
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN_Msk;
  
  GPIOA->MODER &= ~((0b11 << (PITCH_PIN*2)) | (0b11 << (VOLUME_PIN*2)) | (0b11 << (CAL_PIN*2)));

  // Frequency pins to AF, Cal button pin to In, DAC Out to Analogue.
  GPIOA->MODER |= (0b10 << (PITCH_PIN*2)) | (0b10 << (VOLUME_PIN*2))
    | (0b00 << (CAL_PIN*2)) | (0b11 << (DAC_OUT_PIN*2));
  //Pull-up Cal button pin
  GPIOA->PUPDR |= (0b01 << (CAL_PIN*2));

  //Set AFs
  GPIOA->AFR[0] |= 1U << (PITCH_PIN * 4);
  GPIOA->AFR[0] |= 0b10U << (VOLUME_PIN * 4);
}

/** @brief Configures the NVIC with the used IRQs.
 */
static void
_configIRQs (void)
{
  
}

void offsetDacTimer(int32_t offset) {
  int32_t val = INITIAL_DAC_ARR + offset;
  if (val < 1) {
    val = 1;
  } else if (val >= UINT16_MAX) {
    val = UINT16_MAX;
  }
  TIM7->ARR = (uint16_t)val;
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
  configure_gpio();
  _configureDMA();
  configure_dac();
  configure_timers();
}

void
software_reset()
{
  NVIC_SystemReset();
}
