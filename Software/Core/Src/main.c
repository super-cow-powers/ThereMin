/*
  Copyright 2023 David Miall
  
  Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
    Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "main.h"
#include "stm32g051xx.h"

#if defined(STM32L152xE)
#include "stm32l152xe.h"
#include "stm32l1xx_it.h"
#elif defined(STM32G051xx)
#include "stm32g0xx.h"
#include "stm32g0xx_it.h"
#endif

#include "sys-setup.h"
#include "waveform.h"
#include <math.h>
#include <stdint.h>

#define VOL_SCALE_MIN 150


volatile uint32_t SYSTICK_VAL = 0;
static volatile DAC_TypeDef *dac = DAC;
static volatile DMA_Channel_TypeDef *dacDma  = DMA1_Channel1;
static volatile DMAMUX_Channel_TypeDef *dacDmaMux = DMAMUX1_Channel0;

static volatile DMA_TypeDef *dma = DMA1;

static volatile TIM_TypeDef *tim1 = TIM1;
static volatile TIM_TypeDef *tim3 = TIM3;
static volatile TIM_TypeDef *tim15 = TIM15;

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
  volatile int16_t pitchOS = 0;
  int8_t pitchOsSign;
  WF_PROC_DONE = 0;
  ACTIVE_WF_SRC = SINE_WF;
  ACTIVE_WF_BUF = (uint16_t*)WF_BUFF_1;
  INACTIVE_WF_BUF = (uint16_t*)WF_BUFF_2;
  platform_init();
  start_output();

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

      //pitchScale = (int32_t)((double)(2*log((double)pitchOS)));
      double y = ((double)pitchOS * 3 *exp(-0.0003*(double)pitchOS));
      double y_b = (15*exp(-0.006*(double)pitchOS));
      pitchScale = 2*(y + y_b);
      if (pitchScale < 0) pitchScale = 0;
      //pitchScale = pitchOS*2;
      //pitchScale *= pitchOsSign;
      
      for (uint32_t i = 0; i<SAMPLES_NUMBER; i++) {
	int32_t noDC = (ACTIVE_WF_SRC[i] - MIDPOINT);
	int32_t noDcScaled = (noDC*1000) / (volumeScale*10);
	if (volumeScale > 1000) {
	  noDcScaled = 0;
	}
	noDcScaled += MIDPOINT;
	if (noDcScaled > 4095) {
	  noDcScaled = 4095;
	}

	INACTIVE_WF_BUF[i] =  (uint16_t)noDcScaled;
      }
      WF_PROC_DONE = 1;
      }
    //volumeScale = 3;
  }
}

//Pitch/Volume transfer done
void DMA1_Channel2_3_IRQHandler () {
  uint32_t count = 0;
  if (DMA1->ISR & 1U << DMA_ISR_TCIF2_Pos) { //Pitch transfer done
    DMA1->IFCR |= (1U << DMA_IFCR_CTCIF2_Pos) | (1U << DMA_IFCR_CGIF2_Pos);
    for (uint16_t i = 0; i<FRQ_CNT_SAMPLES; i++) {
      count += pitchInputBuffer[i];
    }
    pitchInputMean = count/FRQ_CNT_SAMPLES; //Divide by 32
  }
}

void DMA1_Ch4_7_DMAMUX1_OVR_IRQHandler() {
  uint32_t count = 0;
  if (DMA1->ISR & 1U << DMA_ISR_TCIF4_Pos) { //Volume transfer done
    DMA1->IFCR |= (1U << DMA_IFCR_CTCIF4_Pos) | (1U << DMA_IFCR_CGIF4_Pos);
    for (uint8_t i = 0; i<VOL_CNT_SAMPLES; i++) {
      count += volumeInputBuffer[i];
    }
    volumeInputMean = count/VOL_CNT_SAMPLES; //Divide by 64
  }
}

//Wave transfer done
void DMA1_Channel1_IRQHandler () {
  if (DMA1->ISR & 1U << DMA_ISR_TCIF1_Pos) {
    DMA1->IFCR |= 1U << DMA_IFCR_CTCIF1_Pos;
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


/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  SYSTICK_VAL ++;
}

