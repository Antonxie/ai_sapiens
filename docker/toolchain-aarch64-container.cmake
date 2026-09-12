# aarch64 交叉编译 toolchain (容器内构建版, ai_sapiens -> Jetson Thor)
# 用法: 在 ai_sapiens_dev 容器内 (ubuntu noble, gcc-13 交叉工具链已 apt 安装,
#       sysroot-arm64 挂载于 /sysroot-arm64)
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_SYSROOT /sysroot-arm64)

set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc-13)
set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++-13)
set(CMAKE_ASM_COMPILER /usr/bin/aarch64-linux-gnu-gcc-13)

set(CMAKE_FIND_ROOT_PATH
  /sysroot-arm64
  /sysroot-arm64/opt/ros/jazzy
)

# 容器内 aarch64 系统库 (gcc-13 cross 自带 libstdc++/libc)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
set(Python3_EXECUTABLE /usr/bin/python3.12 CACHE FILEPATH "container python3.12")
set(PYTHON_EXECUTABLE /usr/bin/python3.12 CACHE FILEPATH "container python3.12")
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)

set(CMAKE_LIBRARY_ARCHITECTURE aarch64-linux-gnu)
# rpath-link 不会被 ld 自动加 sysroot 前缀, 必须写带前缀的绝对路径,
# 否则 DT_NEEDED 解析会命中宿主 x86 的 /opt/ros/jazzy 库 (wrong format)。
set(_SYSROOT_LINK_PATH "-Wl,-rpath-link,/sysroot-arm64/opt/ros/jazzy/lib -Wl,-rpath-link,/sysroot-arm64/usr/lib/aarch64-linux-gnu -Wl,-rpath-link,/sysroot-arm64/lib/aarch64-linux-gnu")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${_SYSROOT_LINK_PATH}")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "${_SYSROOT_LINK_PATH}")