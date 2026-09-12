#!/bin/bash
# 免 root 安装 aarch64 交叉工具链 (Ubuntu 官方 deb 解压到 ~/aarch64-cross)
# 来源: 阿里云 Ubuntu 源 (与训练机同源)。GCC 11.4 + arm64 sysroot。
set -e

CROSS_ROOT="${1:-$HOME/aarch64-cross}"
DEB_DIR=$(mktemp -d)
PACKAGES="binutils-aarch64-linux-gnu gcc-aarch64-linux-gnu gcc-11-aarch64-linux-gnu \
cpp-aarch64-linux-gnu cpp-11-aarch64-linux-gnu g++-aarch64-linux-gnu g++-11-aarch64-linux-gnu \
libc6-dev-arm64-cross libc6-arm64-cross libstdc++-11-dev-arm64-cross libstdc++6-arm64-cross \
libgcc-11-dev-arm64-cross libgcc-s1-arm64-cross linux-libc-dev-arm64-cross"

echo "下载 deb ..."
cd "$DEB_DIR"
apt-get download $PACKAGES

echo "解压到 $CROSS_ROOT ..."
mkdir -p "$CROSS_ROOT"
for d in *.deb; do
  dpkg-deb -x "$d" "$CROSS_ROOT"
done

export LD_LIBRARY_PATH="$CROSS_ROOT/usr/lib/x86_64-linux-gnu"
"$CROSS_ROOT/usr/bin/aarch64-linux-gnu-g++" --version | head -1
echo "OK: 工具链已就绪 (source docker/aarch64-cross-env.sh 启用)"