#ifndef SYSSETUP_h
#define SYSSETUP_h
#include <stdint.h>

/** Init the platform */
void platform_init(void);

/** Enable SysTick */
void enable_tick (void);

/** Configure the GPIOs */
void configure_gpio(void);

/** Configure the system timers */
void configure_timers(void);

/** Start the audio output */
void start_output(void);

/** Configure the DAC */
void configure_dac(void);

#endif
