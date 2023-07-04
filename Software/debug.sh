#! /bin/sh
kern=$(uname -a)
gdbCmd=
if [[ kern==*"fedora*" ]]; then
	gdbCmd="gdb"
else
	gdbCmd=arm-none-eabi-gdb
fi

killall openocd
openocd -f /usr/share/openocd/scripts/interface/stlink.cfg -f /usr/share/openocd/scripts/target/stm32g0x.cfg &\
${gdbCmd} ./build/ThereMin.elf -tui -x ./gdb_tgt
