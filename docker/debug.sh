#!/bin/bash

set -x

# make full path
SRC_DIR=$(realpath $(dirname "$0")/..)
echo "Changing directory to $SRC_DIR"
cd $SRC_DIR

# check emdbg installation
if python3 -m pip show emdbg &> /dev/null
then
		echo "emdbg is already installed"
else
		echo "emdbg is not installed, installing..."
		pip3 install emdbg --user --break-system-packages
fi
python3 -m emdbg.debug.openocd -f /usr/share/openocd/scripts/interface/stlink.cfg -f target/stm32h7x.cfg reset
python3 -m emdbg.debug.openocd -f /usr/share/openocd/scripts/interface/stlink.cfg -f target/stm32h7x.cfg upload --source $SRC_DIR/build/px4_fmu-v6x_encrypted_logs/px4_fmu-v6x_encrypted_logs.elf
openocd -f /usr/share/openocd/scripts/interface/stlink.cfg -f /usr/share/openocd/scripts/target/stm32h7x.cfg &
sleep 2
if [ -z "$1" ]
then
		echo "No breakpoint specified, continuing execution"
else
		echo "Breakpoint specified: $1"
fi
BREAKPOINT="break $1"
python3 -m emdbg.debug.gdb --python --elf $SRC_DIR/build/px4_fmu-v6x_encrypted_logs/px4_fmu-v6x_encrypted_logs.elf -ex "monitor reset" -ex "$BREAKPOINT" --ui=cmd remote --port localhost:3333
pkill openocd
