#!/bin/sh

if [ "$1" = "clean" ]; then
    cd $HOME/workspace/PX4-Autopilot_vxworks && make clean
fi

cd $HOME/workspace/PX4-Autopilot_vxworks && make vxworks