#! /bin/sh

openocd -f /usr/share/openocd/scripts/interface/stlink.cfg -f /usr/share/openocd/scripts/target/stm32l0.cfg & arm-none-eabi-gdb ./build/pumpkin.elf -tui -x ./gdb_tgt
killall openocd
