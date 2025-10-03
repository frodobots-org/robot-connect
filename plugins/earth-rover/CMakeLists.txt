cmake_minimum_required(VERSION 3.15)

project(earth-rover)

if(NOT DEFINED ENV{EARTH_ROVER_OPEN_SOURCE_PATH})
  message(FATAL_ERROR "Please set EARTH_ROVER_OPEN_SOURCE_PATH environment variable")
endif()

set(CMAKE_C_COMPILER $ENV{EARTH_ROVER_OPEN_SOURCE_PATH}/Software/Linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin/arm-rockchip830-linux-uclibcgnueabihf-gcc)
set(CMAKE_CXX_COMPILER $ENV{EARTH_ROVER_OPEN_SOURCE_PATH}/Software/Linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin/arm-rockchip830-linux-uclibcgnueabihf-g++)

include_directories(
    ${PROJECT_SOURCE_DIR}/../include
    ${CMAKE_BINARY_DIR}/_deps/json/src/nlohmann_json/include
    $ENV{EARTH_ROVER_OPEN_SOURCE_PATH}/Software/Linux/toolchain/media/include
    $ENV{EARTH_ROVER_OPEN_SOURCE_PATH}/Software/Linux/toolchain/media/include/sample_comm
    $ENV{EARTH_ROVER_OPEN_SOURCE_PATH}/Software/Linux/toolchain/media/include/rkaiq
    $ENV{EARTH_ROVER_OPEN_SOURCE_PATH}/Software/Linux/toolchain/media/include/rkaiq/uAPI2
    $ENV{EARTH_ROVER_OPEN_SOURCE_PATH}/Software/Linux/toolchain/media/include/rkaiq/algos
    $ENV{EARTH_ROVER_OPEN_SOURCE_PATH}/Software/Linux/toolchain/media/include/rkaiq/common
    $ENV{EARTH_ROVER_OPEN_SOURCE_PATH}/Software/Linux/toolchain/media/include/rkaiq/xcore
    $ENV{EARTH_ROVER_OPEN_SOURCE_PATH}/Software/Linux/toolchain/media/include/rkaiq/iq_parser_v2
    $ENV{EARTH_ROVER_OPEN_SOURCE_PATH}/Software/Linux/toolchain/media/include/rkaiq/iq_parser
)

link_directories(
    $ENV{EARTH_ROVER_OPEN_SOURCE_PATH}/Software/Linux/toolchain/media/lib
)

add_library(teleop_ctrl_earthrover SHARED teleop_ctrl_earthrover.cc)
add_library(teleop_media_earthrover SHARED teleop_media_earthrover.cc)
target_link_libraries(teleop_ctrl_earthrover)
target_link_libraries(teleop_media_earthrover
    sample_comm
    rtsp
    rkaiq
    rockiva
    rknnmrt
    rockit_full
    rockchip_mpp
    drm
    rga
)

include(ExternalProject)

ExternalProject_Add(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.2 
    PREFIX ${CMAKE_BINARY_DIR}/_deps/json
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    UPDATE_DISCONNECTED 1
)
add_dependencies(teleop_ctrl_earthrover nlohmann_json)
