#!/bin/sh

export WIND_CC_SYSROOT=$HOME/workspace/fcs_vxworks_bsp/bsp_src/vsb_zynq7k_vxprj

cd "$(dirname "$0")/../.."

if [ "$1" = "clean" ]; then
    make clean

elif [ "$1" = "distclean" ]; then
    make distclean
fi

make yp_fc-v1_default