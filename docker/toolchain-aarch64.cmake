# aarch64 交叉编译 toolchain (ai_sapiens -> Jetson Thor)
# sysroot: /home/anton/sysroot-arm64 (debootstrap arm64 noble + ROS Jazzy arm64 开发包)
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_SYSROOT /home/anton/sysroot-arm64)

set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_ASM_COMPILER aarch64-linux-gnu-gcc)

# 只在 sysroot 内查找 (不要碰宿主 x86 的库/头)
set(CMAKE_FIND_ROOT_PATH
  /home/anton/sysroot-arm64
  /home/anton/sysroot-arm64/opt/ros/jazzy
)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)  # 生成器在 sysroot 内 (arm64, 宿主 binfmt 可执行)

# Python: FindPython3 探测在交叉环境下不可靠 (arm64 解释器无法直接运行)。
# 显式提供宿主 Python 3.10 (含 dev headers + numpy) 的完整结果, 供 rosidl_generator_py 使用;
# 注意: 生成 python 绑定仅辅助编译通过, 实际部署 (C++ onnx) 不依赖 python。
# 宿主必须装有: python3-dev + python3-numpy (见 docker/install_cross_toolchain.sh)
set(Python3_EXECUTABLE /usr/bin/python3.10 CACHE FILEPATH "host python3.10")
set(Python3_FIND_STRATEGY LOCATION CACHE STRING "")
set(Python3_ROOT_DIR /usr CACHE PATH "")
set(Python3_INCLUDE_DIR /usr/include/python3.10 CACHE PATH "")
set(Python3_LIBRARY /usr/lib/x86_64-linux-gnu/libpython3.10.so CACHE FILEPATH "")
set(Python3_INCLUDE_DIRS /usr/include/python3.10 CACHE STRING "")
set(Python3_LIBRARIES /usr/lib/x86_64-linux-gnu/libpython3.10.so CACHE STRING "")
set(Python3_NumPy_INCLUDE_DIR /usr/lib/python3/dist-packages/numpy/core/include CACHE PATH "")
set(NumPy_INCLUDE_DIR /usr/lib/python3/dist-packages/numpy/core/include CACHE PATH "")
set(NumPy_VERSION 1.21.5 CACHE STRING "")
set(PYTHON_EXECUTABLE /usr/bin/python3.10 CACHE FILEPATH "host python3.10")
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
# PACKAGE BOTH: 允许 find_package 在宿主 install/ 找到已装的 workspace 包
# (Config.cmake 的 include 路径会再经 ONLY 模式规整到 sysroot, 无污染)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)

# arm64 库目录钩子 (system libs 在 sysroot/usr/lib/aarch64-linux-gnu)
# 注意: rpath-link 用 sysroot 内相对路径 (ld 会以 --sysroot 前缀), 否则二次映射找不到
set(CMAKE_LIBRARY_ARCHITECTURE aarch64-linux-gnu)
set(_SYSROOT_LINK_PATH "-Wl,-rpath-link,/usr/lib/aarch64-linux-gnu -Wl,-rpath-link,/opt/ros/jazzy/lib")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${_SYSROOT_LINK_PATH}")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "${_SYSROOT_LINK_PATH}")