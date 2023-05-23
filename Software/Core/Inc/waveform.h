#ifndef WAVEFORM_h
#define WAVEFORM_h

#include <stdint.h>
#define SAMPLES_NUMBER 100

extern const uint16_t SINE_WF[SAMPLES_NUMBER];
extern const uint16_t SQUARE_WF[SAMPLES_NUMBER];
extern const uint16_t TRIANGLE_WF[SAMPLES_NUMBER];

extern volatile uint16_t WF_BUFFER[SAMPLES_NUMBER];

#endif
