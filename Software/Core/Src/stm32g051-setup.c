
#include "g0_board_defs.h"
#include "stm32g051xx.h"
#include "stm32g0xx.h"
#include "system_stm32g0xx.h"
#include "sys-setup.h"
#include "waveform.h"
#include <string.h>
#include <stdint.h>


/** @brief Configures the clock to use HSE (external oscillator) and the PLL
   to get SysClk == AHB == APB1 == APB2 == 24MHz */
static void
_configClock (void)
{
  RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN; // Enable SYSCFG clock
  RCC->APBENR1 |= RCC_APBENR1_PWREN; // Enable PWR config clock
  RCC->APBENR1 |= RCC_APBENR1_DBGEN; //Enable Debug Block

  uint32_t pwrVal = PWR->CR1;
  pwrVal &= ~(0b11 << PWR_CR1_VOS_Pos);
  pwrVal |= 0b01 << PWR_CR1_VOS_Pos;
  PWR->CR1 = pwrVal;
  
  FLASH->ACR |= 0b010 << FLASH_ACR_LATENCY_Pos;
  while ((FLASH->ACR & (0b111 << FLASH_ACR_LATENCY_Pos)) != (0b010 << FLASH_ACR_LATENCY_Pos)){

  }
  
  RCC->CR &= ~((uint32_t)1 << RCC_CR_PLLON_Pos); //Disable PLL
  while (RCC->CR & ((uint32_t)1 << RCC_CR_PLLRDY_Pos)); //Wait till PLL is off
  
  RCC->CR |= (uint32_t)1 << RCC_CR_HSION_Pos; //Enable HSI16
  while (!(RCC->CR & (uint32_t)1 << RCC_CR_HSIRDY_Pos)); //Wait till HSI16 is ready

  RCC->PLLCFGR &= ~(0b11);
  RCC->PLLCFGR |= (0b10<<RCC_PLLCFGR_PLLSRC_Pos); //HSI as PLL source
  
  //Set PLL for 128MHz VCO output ((16/2)*16 = 64)
  RCC->PLLCFGR &= ~((uint32_t)0b111 << RCC_PLLCFGR_PLLM_Pos);
  RCC->PLLCFGR |= (uint32_t)0b001 << RCC_PLLCFGR_PLLM_Pos; //Divide HSI16 by 2
  RCC->PLLCFGR &= ~((uint32_t)0b111111 << RCC_PLLCFGR_PLLN_Pos);
  RCC->PLLCFGR |= (uint32_t)16 << RCC_PLLCFGR_PLLN_Pos; //Multiply by 16
  RCC->PLLCFGR |= (uint32_t)0b001 << RCC_PLLCFGR_PLLR_Pos; //PLLR Divide by 2
  RCC->PLLCFGR |= (uint32_t)0b1 << RCC_PLLCFGR_PLLREN_Pos; //Enable PLL R output
  //RCC->CFGR &= ~RCC_CFGR_PLLSRC_Msk;
  
  RCC->CR |= ((uint32_t)0b1 << RCC_CR_PLLON_Pos); //Enable PLL
  while (!(RCC->CR & ((uint32_t)0b1 << RCC_CR_PLLRDY_Pos))); //Wait till PLL is stable
  
  RCC->CFGR |= (0b010 << RCC_CFGR_SW_Pos); //Set source to PLL R
  while ((RCC->CFGR & (0b111 << RCC_CFGR_SWS_Pos)) != (0b010 << RCC_CFGR_SWS_Pos)) {

  }
  SystemCoreClockUpdate();
}

static void _configureDacDMA(void) {
  DMAMUX1_Channel0->CCR |= (uint32_t)25 << DMAMUX_CxCR_DMAREQ_ID_Pos; //Select request 25 (TIM1 UP)
  
  DMA1_Channel1->CCR |= 0b11 << DMA_CCR_PL_Pos; //Max Priority
  DMA1_Channel1->CCR |= 0b01 << DMA_CCR_MSIZE_Pos; //16 Bit data (memory)
  DMA1_Channel1->CCR |= 0b01 << DMA_CCR_PSIZE_Pos; //16 Bit data (peripheral)
  DMA1_Channel1->CCR |= 0b1 << DMA_CCR_MINC_Pos; //Auto-increment memory position
  DMA1_Channel1->CCR |= 0b1 << DMA_CCR_CIRC_Pos; //Circular buffer mode
  DMA1_Channel1->CCR |= 0b1 << DMA_CCR_DIR_Pos; //Read from memory to peripheral
  DMA1_Channel1->CNDTR = SAMPLES_NUMBER; //Set length of data buffer
  DMA1_Channel1->CPAR = (uint32_t)(&(DAC->DHR12R1)); //Data to DAC output
  DMA1_Channel1->CMAR = (uint32_t)((uint16_t*)ACTIVE_WF_BUF); //Data from Waveform Buffer
  DMA1_Channel1->CCR |= 0b1 << DMA_CCR_TCIE_Pos; //Transfer Complete IRQ Enable
  DMA1_Channel1->CCR |= 0b1 << DMA_CCR_EN_Pos; //Enable Channel
  
}

static void _configureVolDMA(void) {
  //Volume Timer DMA
  DMAMUX1_Channel3->CCR |= (uint32_t)43 << DMAMUX_CxCR_DMAREQ_ID_Pos; //Select request 43 (TIM15 UP)
  
  DMA1_Channel4->CCR |= 0b01 << DMA_CCR_PL_Pos; //Medium Priority
  DMA1_Channel4->CCR |= 0b01 << DMA_CCR_MSIZE_Pos; //16 Bit data (memory)
  DMA1_Channel4->CCR |= 0b01 << DMA_CCR_PSIZE_Pos; //16 Bit data (peripheral)
  DMA1_Channel4->CCR |= 0b1 << DMA_CCR_MINC_Pos; //Auto-increment memory position
  DMA1_Channel4->CCR |= 0b1 << DMA_CCR_CIRC_Pos; //Circular buffer mode
  DMA1_Channel4->CCR |= 0b1 << DMA_CCR_TCIE_Pos; //Transfer Complete IRQ Enable
  DMA1_Channel4->CCR &= ~(0b1 << DMA_CCR_DIR_Pos); //Read from peripheral to memory
  DMA1_Channel4->CNDTR = VOL_CNT_SAMPLES; //Set length of data buffer
  DMA1_Channel4->CPAR = (uint32_t)(&(TIM15->CCR1)); //Data from Timer CCR1 output
  DMA1_Channel4->CMAR = (uint32_t)((uint16_t*)volumeInputBuffer); //Data to frequency Buffer
  DMA1_Channel4->CCR |= 0b1 << DMA_CCR_EN_Pos; //Enable Channel
}

static void _configurePitchDMA(void) {
  //Pitch Timer DMA
  DMAMUX1_Channel1->CCR |= (uint32_t)37 << DMAMUX_CxCR_DMAREQ_ID_Pos; //Select request 37 (TIM3 UP)
  
  DMA1_Channel2->CCR |= 0b01 << DMA_CCR_PL_Pos; //Medium Priority
  DMA1_Channel2->CCR |= 0b01 << DMA_CCR_MSIZE_Pos; //16 Bit data (memory)
  DMA1_Channel2->CCR |= 0b01 << DMA_CCR_PSIZE_Pos; //16 Bit data (peripheral)
  DMA1_Channel2->CCR |= 0b1 << DMA_CCR_MINC_Pos; //Auto-increment memory position
  DMA1_Channel2->CCR |= 0b1 << DMA_CCR_CIRC_Pos; //Circular buffer mode
  DMA1_Channel2->CCR |= 0b1 << DMA_CCR_TCIE_Pos; //Transfer Complete IRQ Enable
  DMA1_Channel2->CCR &= ~(0b1 << DMA_CCR_DIR_Pos); //Read from peripheral to memory
  DMA1_Channel2->CNDTR = FRQ_CNT_SAMPLES; //Set length of data buffer
  DMA1_Channel2->CPAR = (uint32_t)(&(TIM3->CCR1)); //Data from Timer CCR1 output
  DMA1_Channel2->CMAR = (uint32_t)((uint16_t*)pitchInputBuffer); //Data to frequency Buffer
  DMA1_Channel2->CCR |= 0b1 << DMA_CCR_EN_Pos; //Enable Channel
}

/** Configure the DMA Engine */
static void _configureDMA(void) {
  RCC->AHBRSTR |= RCC_AHBRSTR_DMA1RST;
  RCC->AHBRSTR &= ~RCC_AHBRSTR_DMA1RST;
  RCC->AHBENR |= RCC_AHBENR_DMA1EN; //Using DMA1
  
  _configureDacDMA();
  _configureVolDMA();
  _configurePitchDMA();
  DMA1->IFCR |= 0xFFFFFFF;
}

static void _configureDacTimer(void) {
  //Timer 1
  RCC->APBRSTR2 |= (RCC_APBRSTR2_TIM1RST);
  RCC->APBRSTR2 &= ~(RCC_APBRSTR2_TIM1RST);
  RCC->APBENR2 |= (RCC_APBENR2_TIM1EN);
  
  TIM1->PSC = 0;
  TIM1->CR1 &= ~( TIM_CR1_OPM | TIM_CR1_UDIS); //Auto-reload preload, multi-pulse mode, enable Update Event
  TIM1->CR1 |= 0b1 << TIM_CR1_URS;
  TIM1->DIER |= TIM_DIER_UDE_Msk; //Enable DMA Request on update
  TIM1->ARR = INITIAL_DAC_ARR;
  DBG->APBFZ2 |= DBG_APB_FZ2_DBG_TIM1_STOP; //Freeze on breakpoint
}

static void _configurePitchTimer(void) {
  //Timer 3
  RCC->APBRSTR1 |= (RCC_APBRSTR1_TIM3RST);
  RCC->APBRSTR1 &= ~(RCC_APBRSTR1_TIM3RST);
  RCC->APBENR1 |= (RCC_APBENR1_TIM3EN);
  
  TIM3->CR2 &= ~(0b1U << TIM_CR2_CCDS_Pos); //DMA on capture compare
  DBG->APBFZ1 |= DBG_APB_FZ1_DBG_TIM3_STOP; //Freeze on breakpoint

  TIM3->CR1 &= ~( TIM_CR1_OPM | TIM_CR1_UDIS); //Auto-reload preload, multi-pulse mode, enable Update Event
  //TIM3->CR1 |= 0b1 << TIM_CR1_URS;
  
  TIM3->CCMR1 |= 0b01 << TIM_CCMR1_CC1S_Pos; //Input 1 to CC1
  TIM3->CCMR1 |= 0b0011 << TIM_CCMR1_IC1F_Pos; //8-cycle Filter
  TIM3->SMCR |= 0b101 << TIM_SMCR_TS_Pos; //Timer Input 1 as Trigger
  TIM3->SMCR |= 0b100U << TIM_SMCR_SMS_Pos; //Slave Reset Mode
  TIM3->CCER &= ~((0b1 << TIM_CCER_CC1P_Pos) | (0b1 << TIM_CCER_CC1NP_Pos)); //Rising Edge
  TIM3->CCMR1 &= ~(0b11 << TIM_CCMR1_IC1PSC_Pos); //No Prescaler
  TIM3->DIER |= TIM_DIER_UDE_Msk; //Enable DMA Request on Update. Get data out when registers update.
  TIM3->CCER |= TIM_CCER_CC1E; //Enable CC1
  TIM3->CR1 |= 1U; //Enable Timer
}

static void _configureVolumeTimer(void) {
  //Timer 15
  
  RCC->APBRSTR2 |= (RCC_APBRSTR2_TIM15RST);
  RCC->APBRSTR2 &= ~(RCC_APBRSTR2_TIM15RST);
  RCC->APBENR2 |= (RCC_APBENR2_TIM15EN);

  TIM15->CR2 &= ~(0b1U << TIM_CR2_CCDS_Pos); //DMA on capture compare
  DBG->APBFZ2 |= DBG_APB_FZ2_DBG_TIM15_STOP; //Freeze on breakpoint

  TIM15->CR1 &= ~( TIM_CR1_OPM | TIM_CR1_UDIS); //Auto-reload preload, multi-pulse mode, enable Update Event
  //TIM15->CR1 |= 0b1 << TIM_CR1_URS;
  
  TIM15->CCMR1 |= 0b01 << TIM_CCMR1_CC1S_Pos; //Input 1 to CC1
  TIM15->CCMR1 |= 0b0011 << TIM_CCMR1_IC1F_Pos; //8-cycle Filter
  TIM15->SMCR |= 0b101 << TIM_SMCR_TS_Pos; //Timer Input 1 as Trigger
  TIM15->SMCR |= 0b100U << TIM_SMCR_SMS_Pos; //Slave Reset Mode
  TIM15->CCER &= ~((0b1 << TIM_CCER_CC1P_Pos) | (0b1 << TIM_CCER_CC1NP_Pos)); //Rising Edge
  TIM15->CCMR1 &= ~(0b11 << TIM_CCMR1_IC1PSC_Pos); //No Prescaler
  TIM15->DIER |= TIM_DIER_UDE_Msk; //Enable DMA Request on Update. Get data out when registers update.
  TIM15->CCER |= TIM_CCER_CC1E; //Enable CC1
  TIM15->CR1 |= 1U; //Enable Timer

}

void configure_timers() {
  _configureDacTimer();
  _configurePitchTimer();
  _configureVolumeTimer();
}

void configure_dac() {
  RCC->APBRSTR1 |= RCC_APBRSTR1_DAC1RST;
  RCC->APBRSTR1 &= ~RCC_APBRSTR1_DAC1RST;
  RCC->APBENR1 |= RCC_APBENR1_DAC1EN;
  DAC->DHR12R1 = 200;
  DAC1->MCR |= (0b010 << DAC_MCR_MODE1_Pos) | (0b010 << DAC_MCR_MODE2_Pos); //No Buffer
  DAC->CR |= DAC_CR_EN1; //No output buffer, enable DAC
}

void start_output() {
  memcpy((uint16_t*)ACTIVE_WF_BUF, SINE_WF, SAMPLES_NUMBER*sizeof(uint16_t)); //Place Sine into output buffer
  TIM1->CR1 |= 0b1;
}

void configure_gpio (void) {
  
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
  RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
  
  GPIOA->MODER &= ~((0b11 << (PITCH_PIN*2)) | (0b11 << (VOLUME_PIN*2)) | (0b11 << (CAL_PIN*2)));

  // Frequency pins to AF, Cal button pin to In, DAC Out to Analogue.
  GPIOA->MODER |= (0b10 << (PITCH_PIN*2)) | (0b10 << (VOLUME_PIN*2))
    | (0b00 << (CAL_PIN*2)) | (0b11 << (DAC_OUT_PIN*2));
  //Pull-up Cal button pin
  GPIOA->PUPDR |= (0b01 << (CAL_PIN*2));

  //Set AFs
  GPIOA->AFR[0] |= 1U << (PITCH_PIN * 4);
  GPIOA->AFR[0] |= 5U << (VOLUME_PIN * 4);
}

/** @brief Configures the NVIC with the used IRQs.
 */
static void
_configIRQs (void)
{
  NVIC_EnableIRQ(TIM15_IRQn);
  NVIC_EnableIRQ(TIM3_IRQn);
  //NVIC_EnableIRQ(TIM7_IRQn);
  //NVIC_EnableIRQ(TIM2_IRQn);
  /*
  NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  NVIC_EnableIRQ(DMA1_Channel2_IRQn);*/
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  NVIC_EnableIRQ(DMA1_Ch4_7_DMAMUX1_OVR_IRQn);

}

void offsetDacTimer(int32_t offset) {
  int32_t val = INITIAL_DAC_ARR + offset;
  if (val < 1) {
    val = 1;
  } else if (val >= UINT16_MAX) {
    val = UINT16_MAX;
  }
  TIM1->ARR = (uint16_t)val;
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
