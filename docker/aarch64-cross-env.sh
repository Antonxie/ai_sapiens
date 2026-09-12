#!/bin/bash
# aarch64 交叉编译环境 (免 root, 由 .deb 解压得到, 见 install_cross_toolchain.sh)
# 用法: source aarch64-cross-env.sh
export CROSS_ROOT=/home/anton/aarch64-cross
export XGCC=$CROSS_ROOT/usr/bin/aarch64-linux-gnu-g++
export XCC=$CROSS_ROOT/usr/bin/aarch64-linux-gnu-gcc
export XSYSROOT=$CROSS_ROOT/usr
export LD_LIBRARY_PATH=$CROSS_ROOT/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}
echo "aarch64 交叉环境就绪:"
echo "  g++ = $($XGCC --version | head -1)"