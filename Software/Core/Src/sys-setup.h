#ifndef SYSSETUP_h
#define SYSSETUP_h
#include <stdint.h>

/** Init the platform */
void platform_init();

/** Enable SysTick */
void enable_tick ();
#endif
