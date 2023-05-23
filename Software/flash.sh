#! /bin/sh
killall openocd
openocd -f /usr/share/openocd/scripts/interface/stlink.cfg -f /usr/share/openocd/scripts/target/stm32l1x_dual_bank.cfg -c 'program ./build/ThereMin.bin 0x08000000 reset exit' 
