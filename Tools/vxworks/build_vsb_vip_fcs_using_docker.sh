#!/bin/sh

cd "$(dirname "$0")/../.."
PX4_AUTOPILOT=${PWD##*/}
echo "Building VSB and VIP of VxWorks in Docker container..."

docker -v
docker exec vxbuild bash -c "~/workspace/$PX4_AUTOPILOT/Tools/vxworks/build_vsb_vip_fcs.sh"

# Copy vxworks kernel to host tftp root
VIP_PROJECT=vip_zynq_fcs_vxprj
cd ~/src/vxbuild/workspace/fcs_vxworks_bsp/bsp_src/$VIP_PROJECT/default
cp zynq-zc702.dtb /tftpboot/
cp uVxWorks /tftpboot/