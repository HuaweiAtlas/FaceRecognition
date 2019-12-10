#!/bin/bash
#if [ -z $DDK_HOME ];then
#    echo "[ERROR] DDK_HOME does not exist! Please set environment variable: export DDK_HOME=<root folder of ddk>"
#    echo "eg:  export DDK_HOME==/home/<ddk install user>/tools/che/ddk/ddk/"
#    exit 0
#fi

#ffmpeg path
FFMPEG_PATH="/your/compiled/ffmpeg/path"

# DDK path
export DDK_HOME=/your/installed/ddk/path
DDK_PATH=$DDK_HOME

CUR_DIR=${PWD}

ProjectRootFolder=${CUR_DIR}/../
DeviceBuildPath=${CUR_DIR}/device/
DeviceCmakePath=${ProjectRootFolder}/src/device
HostBuildPath=${CUR_DIR}/host/
HostCmakePath=${ProjectRootFolder}/src/host
CmakeModule=${ProjectRootFolder}/build/cmake
ArmToolchainPath=$CmakeModule/aarch64.cmake
BuildType="Release" #Release|Debug
CpuCount=$(cat /proc/cpuinfo| grep "processor"| wc -l)

function build_device()
{
    mkdir -p ${DeviceBuildPath}
    cd ${DeviceBuildPath}
    cmake -DCMAKE_BUILD_TYPE=$BuildType \
          -DCMAKE_MODULE_PATH=$CmakeModule \
          -DCMAKE_TOOLCHAIN_FILE=$ArmToolchainPath \
          ${DeviceCmakePath}
    make -j$CpuCount
}

function build_host_x86()
{
    mkdir -p $HostBuildPath
    cd $HostBuildPath
    cmake -DCMAKE_BUILD_TYPE=$BuildType \
          -DCMAKE_MODULE_PATH=$CmakeModule \
          -DFFMPEG_PATH=${FFMPEG_PATH} \
          ${HostCmakePath}
    make -j$CpuCount
}

function build_x86()
{
#build device
    build_device

#build host
    build_host_x86

#copy device library
    if [ ! -d ${ProjectRootFolder}/bin/device ];then
        mkdir -p ${ProjectRootFolder}/bin/device
    fi
    cp $DeviceBuildPath/*.so ${ProjectRootFolder}/bin/device/
    rm -rf $DeviceBuildPath
#copy omp library
    cp ${DDK_PATH}/toolchains/aarch64-linux-gcc6.3/aarch64-linux-gnu/lib64/libgomp.so ${ProjectRootFolder}/bin/device/

#copy host executable to bin
    cp $HostBuildPath/facedemo_main ${ProjectRootFolder}/bin/
    rm -rf $HostBuildPath

#copy om models
    if [ ! -d ${ProjectRootFolder}/bin/data/models ];then
        mkdir -p ${ProjectRootFolder}/bin/data/models
    fi

    cp ${ProjectRootFolder}/depository/models/yolov3tiny/yolov3tiny_b4.om ${ProjectRootFolder}/bin/data/models/FaceDetection.om
    cp ${ProjectRootFolder}/depository/models/resnet18/resnet18_b1.om ${ProjectRootFolder}/bin/data/models/FeatureExtract_b1.om
    cp ${ProjectRootFolder}/depository/models/resnet18/resnet18_b4.om ${ProjectRootFolder}/bin/data/models/FeatureExtract_b4.om
    cp ${ProjectRootFolder}/depository/models/resnet18/resnet18_b8.om ${ProjectRootFolder}/bin/data/models/FeatureExtract_b8.om
    cp ${ProjectRootFolder}/depository/models/resnet10/resnet10_b1.om ${ProjectRootFolder}/bin/data/models/FaceLandmark_b1.om
    cp ${ProjectRootFolder}/depository/models/resnet10/resnet10_b4.om ${ProjectRootFolder}/bin/data/models/FaceLandmark_b4.om
    cp ${ProjectRootFolder}/depository/models/resnet10/resnet10_b8.om ${ProjectRootFolder}/bin/data/models/FaceLandmark_b8.om
#copy config files
    cp ${ProjectRootFolder}/depository/config/setup.config ${ProjectRootFolder}/bin/data/
    cp ${ProjectRootFolder}/depository/config/graph.config ${ProjectRootFolder}/bin/data/
#copy feature lib file
    cp ${ProjectRootFolder}/depository/featurelib ${ProjectRootFolder}/bin/data/ -R
}

function main()
{
    build_x86
}

main
