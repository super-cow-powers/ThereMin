#ifndef WAVEFORM_h
#define WAVEFORM_h

#include <stdint.h>
#define SAMPLES_NUMBER 100
#define MIDPOINT 2048

extern const uint16_t SINE_WF[SAMPLES_NUMBER];
extern const uint16_t SQUARE_WF[SAMPLES_NUMBER];
extern const uint16_t TRIANGLE_WF[SAMPLES_NUMBER];

extern volatile uint16_t * ACTIVE_WF_BUF;

extern volatile uint16_t const * ACTIVE_WF_SRC;

#endif
