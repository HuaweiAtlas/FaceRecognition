cmake_minimum_required(VERSION 2.8)
add_compile_options(-std=c++11 -fstack-protector-all -Wl,-z,relro,-z,noexecstack -fPIC)
project(HelloDavinci)

# ddk
find_package(DDK REQUIRED)
include_directories(${DDK_INCLUDE_DIRS} ${DDK_CEREAL_INCLUDE_DIRS} ${DDK_PROTOBUF_INCLUDE_DIRS} ${DDK_GFLAGS_INCLUDE_DIRS} ${DDK_OPENCV_INCLUDE_DIRS})

if(NOT DEFINED FFMPEG_PATH)
    message(FATAL_ERROR "please use -DFFMPEG_PATH=<>")
endif()
set(FFMPEG_DIR ${FFMPEG_PATH} CACHE INTERNAL "FFMPEG location")
link_directories(${FFMPEG_DIR}/lib)

# common
include_directories(. ./inc/ ../common ${FFMPEG_DIR}/include)

# build for host
# main
file(GLOB SRC cpp/*.cpp)
add_executable(facedemo_main ${SRC})
target_link_libraries(facedemo_main ${DDK_HOST_LIBRARIES} pthread avcodec avdevice avfilter avformat avutil swscale)
