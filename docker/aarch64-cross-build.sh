#!/bin/bash
# ============================================================================
# aarch64 (NVIDIA Jetson Thor) 交叉编译一键脚本
#
# 用途:
#   在 x86 宿主上, 用容器 ai-sapiens:x86_64-gc13 (含 gcc-13-aarch64 交叉工具链)
#   + debootstrap arm64 sysroot, 把 ai_sapiens 工作区交叉编译为 aarch64 产物,
#   并可选在宿主用 qemu-aarch64-static 对产物做冒烟验证。
#
# 前置条件 (本机已备, 复现时需先准备):
#   1. 容器镜像: ai-sapiens:x86_64-gc13        -- gcc-13-aarch64 + x86 ROS2 Jazzy
#   2. sysroot:  /home/anton/sysroot-arm64      -- debootstrap noble + ROS Jazzy arm64 开发包
#   3. 依赖:     deps_onnxruntime_aarch64/      -- aarch64 libonnxruntime (PyPI wheel 提取)
#      (1)(2) 的构建方法见 docker/Dockerfile.aarch64 与 install_cross_toolchain.sh 注释;
#       依赖提取见仓库根 README "构建依赖：ONNX Runtime 预编译库" 章节)
#
# 用法:
#   ./docker/aarch64-cross-build.sh [选项]
#      --build-base DIR   输出目录 (宿主路径, 默认 /tmp/xbuild_arm64)
#      --packages "a b c" 要交叉构建的包 (默认 "ai_sapiens_interfaces ai_sapiens_sim2real")
#      --no-smoke         跳过 qemu 冒烟 (仅构建)
#      --ws DIR           工作区根 (默认脚本同级上一级 = <repo>/)
#      --sysroot DIR      arm64 sysroot 路径 (默认 /home/anton/sysroot-arm64)
#      -h|--help          显示帮助
#
# 流程 (即"配置交叉工具链"的落地产物):
#   容器内: 禁用 rosidl_generator_rs (交叉下 em.py 报错, 只需 C++ 绑定)
#           -> colcon build (toolchain + ROS_CROSS_SKIP_PYTHON=1 + ONNXRUNTIME_ROOT=aarch64)
#   宿主端: qemu-aarch64-static 跑 parallel_kinematics_smoke (兼作 aarch64 冒烟)
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DEFAULT="$(cd "${SCRIPT_DIR}/.." && pwd)"

WS="${WS_DEFAULT}"
XSYSROOT="/home/anton/sysroot-arm64"
IMAGE="ai-sapiens:x86_64-gc13"
BUILD_BASE="/tmp/xbuild_arm64"
PACKAGES="ai_sapiens_interfaces ai_sapiens_sim2real"
RUN_SMOKE=1

usage() {
  sed -n '2,40p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-base) BUILD_BASE="$2"; shift 2 ;;
    --packages)   PACKAGES="$2";  shift 2 ;;
    --no-smoke)   RUN_SMOKE=0;    shift ;;
    --ws)         WS="$2";        shift 2 ;;
    --sysroot)    XSYSROOT="$2";  shift 2 ;;
    -h|--help)    usage; exit 0 ;;
    *) echo "未知参数: $1"; usage; exit 1 ;;
  esac
done

# ---- 前置校验 ---------------------------------------------------------------
for p in "${WS}" "${XSYSROOT}" "${WS}/deps_onnxruntime_aarch64" \
         "${WS}/docker/toolchain-aarch64-container.cmake"; do
  if [[ ! -e "${p}" ]]; then
    echo "错误: 缺少前置路径 ${p}" >&2; exit 1
  fi
done
if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "错误: 缺少容器镜像 ${IMAGE}" >&2; exit 1
fi

echo "==> 交叉构建配置"
echo "    ws        = ${WS}"
echo "    sysroot   = ${XSYSROOT}"
echo "    image     = ${IMAGE}"
echo "    build     = ${BUILD_BASE}"
echo "    packages  = ${PACKAGES}"

# ---- 容器内交叉构建 -----------------------------------------------------------
# 一次性容器 (--rm): 容器内的 rs 禁用等环境改动不落盘, 每条命令重新注入即可.
docker run --rm \
  -v "${WS}:/workspace/ai_sapiens" \
  -v "${XSYSROOT}:/sysroot-arm64" \
  -v /tmp:/tmp \
  -v "${WS}/deps_onnxruntime_aarch64/include/onnxruntime:/usr/local/include/onnxruntime" \
  -w /workspace/ai_sapiens \
  -e ROS_CROSS_SKIP_PYTHON=1 \
  "${IMAGE}" \
  bash -c "
    set -e
    # 禁用 Rust 生成器 (交叉交叉架构下 rosidl_generator_rs 的 em.py 报错)
    rm -rf  /opt/ros/jazzy/share/ament_index/resource_index/rosidl_generator_packages/rosidl_generator_rs
    mv -n /opt/ros/jazzy/share/rosidl_generator_rs /opt/ros/jazzy/share/rosidl_generator_rs.disabled 2>/dev/null || true
    source /opt/ros/jazzy/setup.bash
    colcon build \
      --packages-select ${PACKAGES} \
      --build-base ${BUILD_BASE}/build --install-base ${BUILD_BASE}/install \
      --cmake-args \
        -DCMAKE_TOOLCHAIN_FILE=/workspace/ai_sapiens/docker/toolchain-aarch64-container.cmake \
        -DBUILD_TESTING=OFF \
        -DONNXRUNTIME_ROOT=/workspace/ai_sapiens/deps_onnxruntime_aarch64
  "

echo "==> 交叉构建完成: ${BUILD_BASE}/install"

# ---- qemu 冒烟 (宿主侧) ------------------------------------------------------
if [[ "${RUN_SMOKE}" -eq 1 ]]; then
  SMOKE_BIN="${BUILD_BASE}/install/ai_sapiens_sim2real/lib/ai_sapiens_sim2real/parallel_kinematics_smoke"
  if [[ -x "${SMOKE_BIN}" ]]; then
    echo "==> qemu 冒烟: ${SMOKE_BIN}"
    qemu-aarch64-static -L "${XSYSROOT}" \
      -E LD_LIBRARY_PATH=/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu \
      "${SMOKE_BIN}" && echo "==> qemu 冒烟通过 (exit=$?)" \
      || { echo "错误: qemu 冒烟失败" >&2; exit 1; }
  else
    echo "提示: 未找到 ${SMOKE_BIN}, 跳过 qemu 冒烟" >&2
  fi
fi

echo "==> 全部完成"