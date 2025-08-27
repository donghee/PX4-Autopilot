export WIND_WRTOOL_WORKSPACE=$HOME/src/vxbuild/workspace/fcs_vxworks_bsp/bsp_src
VIP_PROJECT=vip_zynq_fcs_vxprj
VSB_PROJECT=vsb_zynq7k_vxprj

echo "Building VxWorks BSP and VSP for YP FCS"
cd $WIND_WRTOOL_WORKSPACE/$VSB_PROJECT
~/WindRiver/wrenv.linux -- wrtool prj clean
~/WindRiver/wrenv.linux -- wrtool prj build

echo "Building VxWorks BSP and VIP for YP FCS"
cd $WIND_WRTOOL_WORKSPACE/$VIP_PROJECT
~/WindRiver/wrenv.linux -- wrtool prj clean
~/WindRiver/wrenv.linux -- wrtool prj build

echo "Copying VxWorks Image files to /tftpboot"
cd $WIND_WRTOOL_WORKSPACE/$VIP_PROJECT/default
cp zynq-zc702.dtb /tftpboot/
cp uVxWorks /tftpboot/