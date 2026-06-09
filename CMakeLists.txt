cmake_minimum_required(VERSION 3.16)
project(UAV_Swarm_APM CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# MAVLink：使用本仓库 include/mavlink 下的生成头（ardupilotmega 方言，与 ArduPilot 一致）
set(MAVLINK_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/mavlink")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hal
    ${CMAKE_CURRENT_SOURCE_DIR}/include/mavlink_adapter
    ${CMAKE_CURRENT_SOURCE_DIR}/include/swarm
    ${CMAKE_CURRENT_SOURCE_DIR}/include/comm
    ${MAVLINK_INCLUDE_DIR}
)

file(GLOB_RECURSE SOURCES
    "src/*.cpp"
    "src/*.c"
)

add_executable(swarm_node ${SOURCES})

target_link_libraries(swarm_node
    pthread
)

# 鲁班猫0 交叉编译选项（按需启用）
# set(CMAKE_SYSTEM_NAME Linux)
# set(CMAKE_SYSTEM_PROCESSOR aarch64)
# set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
# set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
