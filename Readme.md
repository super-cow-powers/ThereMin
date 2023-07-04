# ThereMin()
Minimal Digital Theremin, based on a pair of NE555p timers and an STM32G0.

![Completed Device](./device.jpg)

## Theory of Operation
The device uses a pair of 555 timers to translate hand-position to frequency, which is measured by the STM32 and then used to modify a generated sine wave.

The 555 timers operate in the usual astable mode except with the capacitor replaced with an "antenna"; this forms a capacitor between you and the metal whose value varies with the gap between you and the antenna, which then varies the frequency of the 555 output.

The STM32 measures the period of the 555's output wave, calculating the offset from the initial state (calibration is done at power-on); the offset from the calibrated value is used to calculate an amplitude coefficient and a pitch offset. The amplitude coefficient divides a full-scale sine wave which is stored in a lookup table. The pitch offset is used to increase or decrease the time between changes of output value (by adjusting a timer's reload-point), thus stretching or compacting the output wave as required - although with an apparently exponential response, which is partially corrected by the firmware.

The signal is fed into a TDA2822 amplifier in single-ended mode (SE gave better stability on veroboard) with the input amplitude controlled by a potentiometer; the amplifier feeds into either an (optional) internal speaker or headphone jack depending on a SPDT switch.

## Hardware
The hardware is designed in KiCad. If you want to build your own, you're free to use the files so long as you don't try and blame me for anything which goes wrong as a result; it'd be nice to keep my name on the board, too.

The devices are fairly cheap - about £12-15 for the basic device with no LEDs or pitch-adjust potentiometer (which haven't been written in) - I used JLC for the boards and sourced most of the parts from DigiKey.

The cases are drilled by hand, using a 13mm bit for the rear cables, a 6.5mm and 7.5mm bit for the front switch and pot, and a 6mm bit for the (four) holes on the top. There's no drilling template, just use a centre-punch in the right place for the hole's purpose and drill. 

### "Antennae"
The Antennae (capacitor plates, really) are not too critical. Mine are made from thin brass rod which will fit into the jacks; any conductor which fits the jacks will work - make it into a loop which looks like a theremin antenna aught, with one "leg" slightly shorter than the other for the non-connected end. The bigger the antenna the more sensitive the response generally speaking, although I've found that there's not much to gain with this device from a huge antenna.

## Firmware
The firmware uses device files from ST Micro and ARM CMSIS, the rest can be blamed on me. The ST HAL is not being used, because it looked like a good idea to not use it when I started...

The software is fairly simple although does make heavy use of the DMA engine and Timers - these are described in the chip reference manual and, briefly, in comments.

### Building
The firmware builds with Make and Arm GCC (I suspect ARMCC6 would work, too, but I'm not going to try it) and should build on most *nix systems - it might work on Windows under WSL or mingw, but you're on your own with that one. For anyone who doesn't want to build the firmware, the latest binary is just in the build directory.

The firmware can be flashed with your favourite flashing tool; a script for an ST-Link V2 with OpenOCD is provided, but I recommend just using STMCubeProg if you're either inexperienced or running Windows. 

-----

This was initially designed as a lab kit for the [York Engineering Society](https://github.com/EngYork).
