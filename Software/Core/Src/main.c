#include "main.h"
#include "stm32l152xe.h"
#include "stm32l1xx_it.h"
#include "sys-setup.h"
#include "waveform.h"
#include <math.h>
#include <stdint.h>

#define VOL_SCALE_MIN 150

const uint16_t logTable[1000] = {0,50,79,100,116,129,140,150,158,166,173,179,185,190,195,200,204,208,212,216,220,223,226,229,232,235,238,240,243,245,248,250,252,254,256,258,260,262,264,266,268,270,271,273,275,276,278,279,281,282,284,285,286,288,289,290,292,293,294,295,297,298,299,300,301,302,303,304,305,306,307,308,309,310,311,312,313,314,315,316,317,318,319,320,320,321,322,323,324,325,325,326,327,328,328,329,330,331,331,332,333,334,334,335,336,336,337,338,338,339,340,340,341,342,342,343,344,344,345,345,346,347,347,348,348,349,349,350,351,351,352,352,353,353,354,354,355,355,356,356,357,357,358,358,359,359,360,360,361,361,362,362,363,363,364,364,365,365,366,366,367,367,367,368,368,369,369,370,370,370,371,371,372,372,373,373,373,374,374,375,375,375,376,376,377,377,377,378,378,378,379,379,380,380,380,381,381,381,382,382,383,383,383,384,384,384,385,385,385,386,386,386,387,387,387,388,388,388,389,389,389,390,390,390,391,391,391,392,392,392,393,393,393,394,394,394,394,395,395,395,396,396,396,397,397,397,397,398,398,398,399,399,399,399,400,400,400,401,401,401,401,402,402,402,402,403,403,403,404,404,404,404,405,405,405,405,406,406,406,406,407,407,407,407,408,408,408,408,409,409,409,409,410,410,410,410,411,411,411,411,412,412,412,412,413,413,413,413,414,414,414,414,415,415,415,415,415,416,416,416,416,417,417,417,417,417,418,418,418,418,419,419,419,419,419,420,420,420,420,420,421,421,421,421,422,422,422,422,422,423,423,423,423,423,424,424,424,424,424,425,425,425,425,425,426,426,426,426,426,427,427,427,427,427,428,428,428,428,428,428,429,429,429,429,429,430,430,430,430,430,431,431,431,431,431,431,432,432,432,432,432,433,433,433,433,433,433,434,434,434,434,434,434,435,435,435,435,435,436,436,436,436,436,436,437,437,437,437,437,437,438,438,438,438,438,438,439,439,439,439,439,439,440,440,440,440,440,440,441,441,441,441,441,441,441,442,442,442,442,442,442,443,443,443,443,443,443,444,444,444,444,444,444,444,445,445,445,445,445,445,445,446,446,446,446,446,446,447,447,447,447,447,447,447,448,448,448,448,448,448,448,449,449,449,449,449,449,449,450,450,450,450,450,450,450,451,451,451,451,451,451,451,452,452,452,452,452,452,452,452,453,453,453,453,453,453,453,454,454,454,454,454,454,454,455,455,455,455,455,455,455,455,456,456,456,456,456,456,456,456,457,457,457,457,457,457,457,457,458,458,458,458,458,458,458,458,459,459,459,459,459,459,459,459,460,460,460,460,460,460,460,460,461,461,461,461,461,461,461,461,462,462,462,462,462,462,462,462,463,463,463,463,463,463,463,463,463,464,464,464,464,464,464,464,464,465,465,465,465,465,465,465,465,465,466,466,466,466,466,466,466,466,466,467,467,467,467,467,467,467,467,467,468,468,468,468,468,468,468,468,468,469,469,469,469,469,469,469,469,469,470,470,470,470,470,470,470,470,470,470,471,471,471,471,471,471,471,471,471,472,472,472,472,472,472,472,472,472,472,473,473,473,473,473,473,473,473,473,473,474,474,474,474,474,474,474,474,474,474,475,475,475,475,475,475,475,475,475,475,476,476,476,476,476,476,476,476,476,476,477,477,477,477,477,477,477,477,477,477,478,478,478,478,478,478,478,478,478,478,478,479,479,479,479,479,479,479,479,479,479,480,480,480,480,480,480,480,480,480,480,480,481,481,481,481,481,481,481,481,481,481,481,482,482,482,482,482,482,482,482,482,482,482,483,483,483,483,483,483,483,483,483,483,483,484,484,484,484,484,484,484,484,484,484,484,484,485,485,485,485,485,485,485,485,485,485,485,486,486,486,486,486,486,486,486,486,486,486,486,487,487,487,487,487,487,487,487,487,487,487,487,488,488,488,488,488,488,488,488,488,488,488,488,489,489,489,489,489,489,489,489,489,489,489,489,490,490,490,490,490,490,490,490,490,490,490,490,491,491,491,491,491,491,491,491,491,491,491,491,491,492,492,492,492,492,492,492,492,492,492,492,492,493,493,493,493,493,493,493,493,493,493,493,493,493,494,494,494,494,494,494,494,494,494,494,494,494,494,495,495,495,495,495,495,495,495,495,495,495,495,495,495,496,496,496,496,496,496,496,496,496,496,496,496,496,497,497,497,497,497,497,497,497,497,497,497,497,497,497,498,498,498,498,498,498,498,498,498,498,498
};

volatile uint32_t SYSTICK_VAL = 0;

// DAC on PA4
static volatile TIM_TypeDef *tim = TIM7;
static volatile TIM_TypeDef *tim2 = TIM2;
static volatile TIM_TypeDef *tim3 = TIM3;

static volatile DAC_TypeDef *dac = DAC;
static volatile DMA_Channel_TypeDef* dacDma = DMA1_Channel3;

volatile uint16_t volumeInputBuffer[VOL_CNT_SAMPLES];
volatile uint16_t pitchInputBuffer[FRQ_CNT_SAMPLES];

volatile uint16_t WF_BUFF_1[SAMPLES_NUMBER];
volatile uint16_t WF_BUFF_2[SAMPLES_NUMBER];

volatile uint16_t *ACTIVE_WF_BUF;
volatile uint16_t *INACTIVE_WF_BUF;
volatile uint16_t const *ACTIVE_WF_SRC;
volatile uint8_t WF_PROC_DONE;

volatile uint16_t volumeInputMean;
volatile int32_t volumeScale;
volatile int32_t pitchScale;
volatile uint16_t pitchInputMean;

volatile uint16_t volumeCal;
volatile uint16_t pitchCal;



/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  int16_t lastPitchOs = 0;
  int16_t pitchOS = 0;
  int8_t pitchOsSign;
  WF_PROC_DONE = 0;
  ACTIVE_WF_SRC = SINE_WF;
  ACTIVE_WF_BUF = (uint16_t*)WF_BUFF_1;
  INACTIVE_WF_BUF = (uint16_t*)WF_BUFF_2;
  platform_init();
  start_output();
  //NVIC_EnableIRQ(TIM7_IRQn);
  //NVIC_EnableIRQ(TIM2_IRQn);
  NVIC_EnableIRQ(TIM3_IRQn);
  NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  for (uint32_t i = 0; i<3*UINT16_MAX; i++) {

  }
  volumeCal = volumeInputMean;
  pitchCal = pitchInputMean;
  while (1) {

    if (!WF_PROC_DONE){
      //Calculate scaled number by multiplying
      volumeScale = (volumeInputMean - volumeCal);
      if (volumeScale < 10) {
	volumeScale = 10;
      }
      volumeScale += 100;
      //Try leaving pitch-scaling as-is?
      pitchOS = pitchInputMean - pitchCal;
      lastPitchOs = pitchOS;
      pitchOsSign = 1;
      if (pitchOS < 0) {
	pitchOsSign = -1;
	pitchOS = 1;
      } else if (pitchOS == 0) {
	pitchOsSign = 0;
      }
      
      //pitchScale = (int32_t)((double)(30*log(pitchOS*50)));
      pitchScale = pitchOS*1.3;
      //pitchScale *= pitchOsSign;
      
      for (uint32_t i = 0; i<SAMPLES_NUMBER; i++) {
	int32_t noDC = (ACTIVE_WF_SRC[i] - MIDPOINT);
	int32_t noDcScaled = ((noDC*10000) / (volumeScale))/100;
	noDcScaled += MIDPOINT;
	INACTIVE_WF_BUF[i] =  (uint16_t)noDcScaled;
      }
      WF_PROC_DONE = 1;
      }
    //volumeScale = 3;
  }
}

void DMA1_Channel2_IRQHandler () {
  uint32_t count = 0;
  if (DMA1->ISR & 1U << DMA_ISR_TCIF2_Pos) {
    DMA1->IFCR |= 1U << DMA_IFCR_CTCIF2_Pos;
    for (uint16_t i = 0; i<FRQ_CNT_SAMPLES; i++) {
      count += pitchInputBuffer[i];
    }
    pitchInputMean = count >> 5; //Divide by 32
  }
}

void DMA1_Channel6_IRQHandler () {
  uint32_t count = 0;
  if (DMA1->ISR & 1U << DMA_ISR_TCIF6_Pos) {
    DMA1->IFCR |= 1U << DMA_IFCR_CTCIF6_Pos;
    for (uint8_t i = 0; i<VOL_CNT_SAMPLES; i++) {
      count += volumeInputBuffer[i];
    }
    volumeInputMean = count >> 6; //Divide by 64
  }
}

void DMA1_Channel3_IRQHandler () {
  if (DMA1->ISR & 1U << DMA_ISR_TCIF3_Pos) {
    DMA1->IFCR |= 1U << DMA_IFCR_CTCIF3_Pos;
    if (WF_PROC_DONE) {
      DMA1_Channel3->CMAR = (uint32_t)((uint16_t*)INACTIVE_WF_BUF);
      uint16_t* tempWfBuf = (uint16_t*)ACTIVE_WF_BUF;
      ACTIVE_WF_BUF = INACTIVE_WF_BUF;
      INACTIVE_WF_BUF = tempWfBuf;
      offsetDacTimer(-pitchScale); //Apply in TCIRQ for synch
      WF_PROC_DONE = 0;
      }
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

