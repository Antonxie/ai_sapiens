# ROBOTIS AI Sapiens ROS 2 Packages

This repository contains the official ROS 2 packages for the ROBOTIS AI Sapiens K1 platform. These packages provide the robot description, bringup configuration, controller interfaces, and simulation-to-real tools used to control the robot and develop Physical AI applications.

For product information and documentation, visit:
  - [AI Sapiens](https://www.robotis.com/en/product/ecosystem-aisapiens.php)
  - [Documentation for AI Sapiens](https://docs.robotis.com/docs/systems/aisapiens/introduction)

To learn more about the Physical AI Tools, visit:
  - [Physical AI Tools](https://github.com/ROBOTIS-GIT/physical_ai_tools)

To explore open-source platforms in a simulation environment, visit:
  - [Simulation Models](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie)

For usage instructions and demonstrations, check out:
  - [Tutorial Videos](https://www.youtube.com/@ROBOTISOpenSourceTeam)

To access datasets and pre-trained models for ROBOTIS open-source platforms, see:
  - [AI Models & Datasets](https://huggingface.co/ROBOTIS)

To use the Docker image for running ROS packages and Physical AI tools, visit:
  - [Docker Images](https://hub.docker.com/r/robotis/ros/tags)

---

## 构建依赖：ONNX Runtime 预编译库（x86-64 / Jetson Thor aarch64）

`ai_sapiens_sim2real` 的策略推理节点使用 ONNX Runtime C/C++ API 加载 `policy.onnx`
（384 维观测 -> 29 维动作）。项目**不随仓库分发** ONNX Runtime 二进制
（见 `.gitignore` 的 `deps_onnxruntime/` 与 `deps_onnxruntime_aarch64/`），
需要按本机架构自行准备两种版本之一。官方 GitHub release 在部分网络环境下
下载困难，下面的步骤均使用 **PyPI wheel 提取**，国内网络可用。

### 1. 两种架构要准备的东西

| 架构 | 目标设备 | 库文件 | 用途 |
|---|---|---|---|
| x86-64 | 本机开发 / 仿真 PC（Ubuntu 24.04 + ROS2 Jazzy） | `libonnxruntime.so.1.x.x`（~19 MiB，x86-64 ELF） | 本机跑 sim2sim |
| aarch64 | NVIDIA Jetson Thor（Ubuntu 24.04） | `libonnxruntime.so.1.x.x`（aarch64 ELF，GLIBC <= 2.27 兼容） | 真机部署 |

仓库内参考版本：**1.23.2**（`deps_onnxruntime/lib/libonnxruntime.so -> libonnxruntime.so.1.23.2`）。
可自行换成其它 1.x 版本，只要保证 `ai_sapiens_sim2real/CMakeLists.txt` 的
`ONNXRUNTIME_ROOT` 指向的目录包含相同的 6 个头文件与 `libonnxruntime.so`。

### 2. 目录布局（提取后）

```
deps_onnxruntime/            # x86-64 版本（或 deps_onnxruntime_aarch64/ 给 Thor）
├── include/onnxruntime/     # 必须包含全部 6 个头文件
│   ├── onnxruntime_c_api.h
│   ├── onnxruntime_cxx_api.h
│   ├── onnxruntime_cxx_inline.h   # ← 最容易缺，缺了编译报 "No such file"
│   ├── onnxruntime_ep_c_api.h
│   ├── onnxruntime_float16.h
│   └── onnxruntime_session_options_config_keys.h
└── lib/
    ├── libonnxruntime.so -> libonnxruntime.so.1.23.2
    ├── libonnxruntime.so.1 -> libonnxruntime.so.1.23.2
    ├── libonnxruntime.so.1.23.2
    └── libonnxruntime_providers_shared.so
```

### 3. 从 PyPI wheel 提取（x86-64 示例）

```bash
# 1) 下载 wheel（--no-deps 只要主包；纯 wheel 本质是 zip）
python3 -m pip download onnxruntime==1.23.2 --no-deps -d /tmp/ort_dl

# 2) 解压到临时目录
mkdir -p /tmp/ort_x && cd /tmp/ort_x
unzip /tmp/ort_dl/onnxruntime-1.23.2-*.whl

# 3) 组装 deps_onnxruntime/（在仓库根执行）
mkdir -p deps_onnxruntime/include deps_onnxruntime/lib
cp /tmp/ort_x/onnxruntime/capi/*.h            deps_onnxruntime/include/onnxruntime/
# wheel 内头文件在 capi/ 下；如果缺少 onnxruntime_cxx_inline.h 等，
# 从 GitHub 仓库 raw 补：
#   https://raw.githubusercontent.com/microsoft/onnxruntime/v1.23.2/include/onnxruntime/onnxruntime_cxx_inline.h
cp /tmp/ort_x/onnxruntime/capi/libonnxruntime.so.1.23.2  deps_onnxruntime/lib/
cp -P /tmp/ort_x/onnxruntime/capi/libonnxruntime.so*      deps_onnxruntime/lib/ 2>/dev/null || true
# 若 wheel 内没有符号链接，手动建：
cd deps_onnxruntime/lib
ln -sf libonnxruntime.so.1.23.2 libonnxruntime.so.1
ln -sf libonnxruntime.so.1.23.2 libonnxruntime.so

# 4) 校验架构（必须是 x86-64）
file deps_onnxruntime/lib/libonnxruntime.so.1.23.2   # -> ELF 64-bit LSB shared object, x86-64
```

aarch64（Thor）只需把 wheel 换成 `onnxruntime-1.23.2-cp*-cp*-linux_aarch64.whl`
（PyPI 平台标签 `manylinux_2_27_aarch64`，GLIBC <= 2.27 兼容 Jetson 的 U-Boot Ubuntu），
重复 2-3 步组装到 `deps_onnxruntime_aarch64/`，并校验 `ELF ... aarch64`。

### 4. 构建时如何指向依赖

| 场景 | 命令 |
|---|---|
| x86 本机 colcon | `colcon build --packages-select ai_sapiens_interfaces ai_sapiens_sim2real --cmake-args -DONNXRUNTIME_ROOT=$PWD/deps_onnxruntime` |
| aarch64 交叉编译（容器） | `docker run -e ROS_CROSS_SKIP_PYTHON=1 ... --cmake-args -DCMAKE_TOOLCHAIN_FILE=docker/toolchain-aarch64-container.cmake -DONNXRUNTIME_ROOT=/workspace/ai_sapiens/deps_onnxruntime_aarch64` |

`CMakeLists.txt` 通过 `find_path(onnxruntime_cxx_api.h HINTS ${ONNXRUNTIME_ROOT}/include)`
与 `find_library(onnxruntime HINTS ${ONNXRUNTIME_ROOT}/lib)` 定位依赖；
`ONNXRUNTIME_ROOT` 未传时回退到 `/usr/local/include/onnxruntime`（需完整 6 个头 +
`/usr/local/lib/libonnxruntime.so`）。

### 5. 常见问题排查

| 症状 | 原因 | 解决 |
|---|---|---|
| `fatal error: onnxruntime_cxx_inline.h: No such file` | 头文件不全（常见于 GitHub release 包或旧 wheel） | 补全 6 个头（见 §3 第 3 步注释） |
| `file in wrong format` / 链接报架构错 | 把 x86 库传给了 aarch64 交叉构建（或反之） | `file libonnxruntime.so*` 核对 ELF 架构 |
| `GLIBC_2.38 not found` | 宿主交叉工具链过旧（GCC 11 eg.） | 用容器内 `gcc-13-aarch64-linux-gnu`（`ai-sapiens:x86_64-gc13` 镜像已带） |
| qemu 冒烟乱码 / `Invalid input name` | 测试程序里 `GetInputNameAllocated().get()` 临时对象悬垂 | 保留 `AllocatedStringPtr` 生命周期（详见 `docs/development-notes-2026-09-12.md` §1） |
