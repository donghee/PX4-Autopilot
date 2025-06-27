#!/bin/sh

export WIND_CC_SYSROOT=$HOME/workspace/vsb_zynq7k_vxprj

if [ "$1" = "clean" ]; then
    cd $HOME/workspace/PX4-Autopilot_vxworks && make clean
fi

cd $HOME/workspace/PX4-Autopilot_vxworks && make yp_fc-v1_default