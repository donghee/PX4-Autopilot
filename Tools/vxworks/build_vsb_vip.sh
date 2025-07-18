export WIND_WRTOOL_WORKSPACE=$HOME/src/vxbuild/workspace

echo "Building VxWorks BSP and VIP for Zynq-7000"
cd $WIND_WRTOOL_WORKSPACE/vsb_zynq7k_vxprj
~/WindRiver/wrenv.linux -- wrtool prj clean
~/WindRiver/wrenv.linux -- wrtool prj build

echo "Building VxWorks BSP and VIP for Zynq-7000"
cd $WIND_WRTOOL_WORKSPACE/vip_zynq7k_vxprj
~/WindRiver/wrenv.linux -- wrtool prj clean
~/WindRiver/wrenv.linux -- wrtool prj build

echo "Copying VxWorks Image files to /tftpboot"
cd $WIND_WRTOOL_WORKSPACE/vip_zynq7k_vxprj/default
cp zynq-zc702.dtb /tftpboot/
cp uVxWorks /tftpboot/