### ThereMin()
Minimal Digital Theremin, based on a pair of NE555p timers and an STM32G0.

![Completed Device](https://github.com/super-cow-powers/ThereMin/blob/master/complete\ device.jpg)

## Theory of Operation
The device uses a pair of 555 timers to translate hand-position to frequency, which is measured by the STM32 and then used to modify a generated sine wave.

The 555 timers operate in the usual astable mode except with the capacitor replaced with an "antenna"; this forms a capacitor between you and the metal whose value varies with the gap between you and the antenna, which then varies the freqency of the 555 output.

The STM32 measures the period of the 555's output wave, calculating the offset from the initial state (calibration is done at power-on); the offset from the calibrated value is used to calculate an amplitude coefficient and a pitch ofset. The amplitude coefficient divides a full-scale sine wave which is stored in a lookup table. The pitch offset is used to increase or decrease the time between changes of output value (by adjusting a timer's reload-point), thus stretching or compacting the output wave as required - although with an apparently exponential response, which is partially corrected by the firmware. 

## Hardware
The hardware is designed in KiCad. If you want to build your own, you're free to use the files so long as you don't try and blame me for anything which goes wrong as a result; it'd be nice to keep my name on the board, too.

The devices are fairly cheap - about £12-15 for the basic device with no LEDs or pitch-adjust potentiometer (which haven't been written in) - I used JLC for the boards and sourced most of the parts from DigiKey.

## Firmware
The firmware uses device files from ST Micro and ARM CMSIS, the rest can be blamed on me. The ST HAL is not being used, because it looked like a good idea to not use it when I started...

The software is fairly simple although does make heavy use of the DMA engine and Timers - these are described in the chip reference manual and, briefly, in comments.

-----

This was initially designed as a lab kit for the [York Engineering Society](https://github.com/EngYork).
