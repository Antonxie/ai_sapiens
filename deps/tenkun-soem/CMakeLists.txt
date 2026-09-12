# Tenkun EtherCAT(SOEM) 通讯工程 — 与 RSP 主工程解耦，网关 + CiH408 协议栈。
cmake_minimum_required(VERSION 3.16)
project(tenkun_soem_standalone LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_STANDARD 99)

set(TENKUN_SOEM_SLAVE_NUMBER 4 CACHE STRING "逻辑 EtherCAT 从站数上限（预处理器宏 SLAVE_NUMBER）")

# ---------- SOEM（默认使用本工程内 third_party/SOEM，与主仓库解耦）----------
set(TENKUN_SOEM_SOEM_ROOT "" CACHE PATH "覆盖 SOEM 源码路径；为空则使用 third_party/SOEM")
if(TENKUN_SOEM_SOEM_ROOT AND EXISTS "${TENKUN_SOEM_SOEM_ROOT}/CMakeLists.txt")
  add_subdirectory("${TENKUN_SOEM_SOEM_ROOT}" "${CMAKE_BINARY_DIR}/soem_build")
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/SOEM/CMakeLists.txt")
  add_subdirectory(third_party/SOEM)
else()
  message(FATAL_ERROR
    "未找到 SOEM。请将 SOEM 置于本目录 third_party/SOEM，或执行：\n"
    "  cmake -DTENKUN_SOEM_SOEM_ROOT=/path/to/SOEM ...")
endif()

find_package(Boost REQUIRED)

# tenkun/：仅 CiH408 CAN FD 组包/解析（无 PDO）；tenkun_link/：网关 PDO + SOEM 衔接。
set(TENKUN_COMM_SRCS
  src/tenkun_rti.cpp
  src/soem_transmit/transmit_soem_core.cpp
  src/tenkun/tenkun_cih408.c
  src/tenkun/tenkun_canfd_parse.c
  src/tenkun_link/tenkun_gateway_pdo.c
  src/tenkun_link/tenkun_slave_station.cpp 
)

add_library(tenkun_soem_comm STATIC ${TENKUN_COMM_SRCS})
target_compile_definitions(tenkun_soem_comm PUBLIC "SLAVE_NUMBER=${TENKUN_SOEM_SLAVE_NUMBER}")
target_include_directories(tenkun_soem_comm PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/soem_transmit"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/tenkun_link"
)
target_include_directories(tenkun_soem_comm BEFORE PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/src/tenkun"
)
target_include_directories(tenkun_soem_comm PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(tenkun_soem_comm PUBLIC soem PRIVATE ${Boost_LIBRARIES})
if(UNIX AND NOT APPLE)
  target_link_libraries(tenkun_soem_comm PUBLIC pthread)
endif()

add_executable(tenkun_soem_demo demo_main.cpp)
target_link_libraries(tenkun_soem_demo PRIVATE tenkun_soem_comm ${Boost_LIBRARIES})

add_executable(tenkun_soem_read_states demo_read_states.cpp)
target_link_libraries(tenkun_soem_read_states PRIVATE tenkun_soem_comm ${Boost_LIBRARIES})

add_executable(tenkun_soem_zero_calibrate demo_zero_calibrate.cpp)
target_link_libraries(tenkun_soem_zero_calibrate PRIVATE tenkun_soem_comm ${Boost_LIBRARIES})
