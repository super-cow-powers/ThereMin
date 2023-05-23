#! /bin/sh
uname=$(uname -a)

if [[ $uname == *"fedora"* ]]; then
    DEB="gdb"
else
    DEB="arm-none-eabi-gdb"
fi

openocd -f /usr/share/openocd/scripts/interface/stlink.cfg -f /usr/share/openocd/scripts/target/stm32l1.cfg & $DEB ./build/ThereMin.elf -tui -x ./gdb_tgt

killall openocd
