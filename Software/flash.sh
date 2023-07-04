#! /bin/sh
killall openocd
openocd -f /usr/share/openocd/scripts/interface/stlink.cfg -f /usr/share/openocd/scripts/target/stm32g0x.cfg -c 'program ./build/ThereMin.bin 0x08000000 verify reset exit' 
