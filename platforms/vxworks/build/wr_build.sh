###################################################################################
# Wind River Workbench generated wr_build.sh.
# Do not edit!!!
#
# Copyright (c) 2020 Wind River Systems, Inc. All Rights Reserved.
###################################################################################

## update path when cmake is MSYS cmake.
FINAL_VSB_PATH=/home/donghee/workspace/vsb_zynq7k_vxprj
IS_MSYS2=$(echo `which cmake` | grep -i "msys2")
if [ "$IS_MSYS2" != "" ];
then
  FINAL_VSB_PATH=/$(echo "$FINAL_VSB_PATH" | sed "s/://")
  MSYS2_PATH=`dirname $IS_MSYS2`
  export PATH=$MSYS2_PATH:$PATH
fi

echo `cmake --version`

## for arguments.
IS_DEBUG=$(echo $* | grep "DEBUG_MODE=1")
IS_REBUILD=$(echo $* | grep "clean")
IS_INSTALL=$(echo $* | grep "install")
IS_JOBS=$(echo $* | grep "JOBS")

if [ "$IS_DEBUG" = "" ];
then
  BUILD_TYPE=Release
else
  BUILD_TYPE=Debug
fi

if [ "$IS_REBUILD" != "" ];
then
  echo "- Remove directory $BUILD_TYPE"; rm -rf $BUILD_TYPE
  exit
fi

if [ "$IS_INSTALL" = "" ];
then
  INSTALL=""
else
  INSTALL=install
fi

if [ "$IS_JOBS" = "" ];
then
  JOBS=""
else
  JOBS=$(echo $* | sed "s/.*JOBS=/-j/")
fi

pwd

mkdir -p $BUILD_TYPE && cd $BUILD_TYPE
mkdir -p .cmake/api/v1/query && touch .cmake/api/v1/query/codemodel-v2 


export WIND_CC_SYSROOT=$FINAL_VSB_PATH

cmake \
 \
 \
 \
 \
-G "Unix Makefiles" \
-DCMAKE_VERBOSE_MAKEFILE=ON \
-DCMAKE_BUILD_TYPE=$BUILD_TYPE \
-DWIND_CC_SYSROOT=$FINAL_VSB_PATH \
-DCMAKE_TOOLCHAIN_FILE=$FINAL_VSB_PATH/mk/toolchain.cmake \
-DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
-DVX_TARGET_TYPE=DKM  \
../..

echo "[checkpoint] make $INSTALL $JOBS"

make $INSTALL $JOBS