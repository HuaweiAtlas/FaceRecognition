cmake_minimum_required(VERSION 2.8)
add_compile_options(-std=c++11 -fopenmp -fPIC -DCPU_ONLY -march=armv8-a -DENABLE_NEON=ON -fstack-protector-all -Wl,-z,relro,-z,noexecstack)
project(HelloDavinci)

# ddk
find_package(DDK REQUIRED)
include_directories(${DDK_INCLUDE_DIRS} ${DDK_CEREAL_INCLUDE_DIRS} ${DDK_PROTOBUF_INCLUDE_DIRS} ${DDK_OPENCV_INCLUDE_DIRS})

# common
include_directories(./common/inc ../common/)

# build for device
# FaceDetectionEngine
aux_source_directory(FaceDetectionEngine FaceDetectionEngine_SRC)
add_library(FaceDetectionEngine SHARED ${FaceDetectionEngine_SRC})
target_include_directories(FaceDetectionEngine PUBLIC FaceDetectionEngine)
target_link_libraries(FaceDetectionEngine ${DDK_DEVICE_LIBRARIES})


# FaceFeatureEngine
aux_source_directory(FaceFeatureEngine FaceFeatureEngine_SRC)
add_library(FaceFeatureEngine SHARED ${FaceFeatureEngine_SRC})
target_include_directories(FaceFeatureEngine PUBLIC FaceFeatureEngine)
target_link_libraries(FaceFeatureEngine ${DDK_DEVICE_LIBRARIES})

# WarpAffineEngine
aux_source_directory(WarpAffineEngine WarpAffineEngine_SRC)
add_library(WarpAffineEngine SHARED ${WarpAffineEngine_SRC})
target_include_directories(WarpAffineEngine PUBLIC WarpAffineEngine)
target_link_libraries(WarpAffineEngine ${DDK_DEVICE_LIBRARIES})

# VdecEngine
aux_source_directory(VdecEngine VdecEngine_SRC)
add_library(VdecEngine SHARED ${VdecEngine_SRC})
target_include_directories(VdecEngine PUBLIC VdecEngine)
target_link_libraries(VdecEngine ${DDK_DEVICE_LIBRARIES})

# FaceLandmarkEngine
aux_source_directory(FaceLandmarkEngine FaceLandmarkEngine_SRC)
add_library(FaceLandmarkEngine SHARED ${FaceLandmarkEngine_SRC})
target_include_directories(FaceLandmarkEngine PUBLIC FaceLandmarkEngine)
target_link_libraries(FaceLandmarkEngine ${DDK_DEVICE_LIBRARIES})

# SORTEngine
aux_source_directory(SORTEngine SORTEngine_SRC)
add_library(SORTEngine SHARED ${SORTEngine_SRC})
target_include_directories(SORTEngine PUBLIC SORTEngine)
target_link_libraries(SORTEngine ${DDK_DEVICE_LIBRARIES})

# DistributionEngine
aux_source_directory(DistributionEngine DistributionEngine_SRC)
add_library(DistributionEngine SHARED ${DistributionEngine_SRC})
target_include_directories(DistributionEngine PUBLIC DistributionEngine)
target_link_libraries(DistributionEngine ${DDK_DEVICE_LIBRARIES})
