cmake_minimum_required(VERSION 3.16)
project(real_robot_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# ---- tenkun-soem (SOEM + CiH408 + tenkun_rti 全身调度), 本地 vendored ----
get_filename_component(TENKUN_SOEM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../deps/tenkun-soem" ABSOLUTE)
get_filename_component(SIM2REAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(SOEM_DIR ${TENKUN_SOEM_DIR}/third_party/SOEM)
file(GLOB TENKUN_C_SRC ${TENKUN_SOEM_DIR}/src/tenkun/*.c)
set(TENKUN_CPP_SRC
  ${TENKUN_SOEM_DIR}/src/tenkun_rti.cpp
  ${TENKUN_SOEM_DIR}/src/soem_transmit/transmit_soem_core.cpp
)
file(GLOB TENKUN_LINK_CPP ${TENKUN_SOEM_DIR}/src/tenkun_link/*.cpp)

# ---- 并行解算 (与 ai_sapiens_sim2real 同源码, 直接编译避免跨包导出) ----
set(PARALLEL_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/../../ai_sapiens_sim2real/src/parallel/parallel_kinematics.cpp
)

add_library(${PROJECT_NAME} SHARED
  src/real_robot_system.cpp
  ${SOEM_SRC}
  ${TENKUN_C_SRC}
  ${TENKUN_CPP_SRC}
  ${TENKUN_LINK_CPP}
  ${PARALLEL_SRC}
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_definitions(${PROJECT_NAME} PRIVATE
  # SOEM 需要 POSIX 实时接口
  _POSIX_C_SOURCE=200809L
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# tenkun-soem 头仅在编译期使用, 用 PRIVATE 避免导出源树外路径
target_include_directories(${PROJECT_NAME} PRIVATE
  ${TENKUN_SOEM_DIR}/src
  ${TENKUN_SOEM_DIR}/src/tenkun
  ${TENKUN_SOEM_DIR}/src/tenkun_link
  ${TENKUN_SOEM_DIR}/src/soem_transmit
  ${SOEM_DIR}/soem
  ${SOEM_DIR}/osal
  ${SOEM_DIR}/osal/linux
  ${SOEM_DIR}/oshw
  ${SOEM_DIR}/oshw/linux
  ${SIM2REAL_DIR}/ai_sapiens_sim2real/include
  /usr/include/eigen3
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  yaml-cpp
  Eigen3
)
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} Threads::Threads rt)

pluginlib_export_plugin_description_file(
  hardware_interface real_robot_hardware_interface.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  yaml-cpp
  Eigen3
)
ament_package()