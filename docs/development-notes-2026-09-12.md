# 0624 sim2real 开发笔记（2026-09-12）

> 会话主题三连：① qemu 冒烟测试 sim2real_node → ② 真机 harness 通信协议适配方案 →
> ③ 并联解算（腰 roll/pitch + 左右踝 pitch/roll）移植到 x86/arm64 双架构 sim2real 系统并通过双交叉冒烟。
> 本文从会话起点到每步改动、失败原因、修正与回退逐条记录，可作为回放与教学材料。

---

## 0. 起点（会话开始时的源码状态）

- **git 基线**：`65718c1`（Merge ROBOTIS feature-teleop-velocity-command-timeout）
- 会话开始前工作区已有**未提交改动**（历史会话遗留，与本次任务相关的部分）：
  - `ai_sapiens_mujoco`：`mujoco_simulation.{hpp,cpp}`、`mujoco_viewer.cpp` 修改
    （mujoco_vendor 0.1.0 API 适配、帧记录死锁修复等）
  - `ai_sapiens_hardware_interfaces`：`mujoco_hardware_interface.{hpp,cpp}` 修改
  - `ai_sapiens_sim2real`：`mode_controller.cpp`、`observations.cpp`、`policy_runtime.cpp`、
    `config/teleop/keyboard.yaml` 修改；新增 `config/modelae_0624_config.yaml`、`config/teleop/modelae_0624_keyboard.yaml`
  - `ai_sapiens_bringup/launch/modelae_0624_hw.launch.py`、`modelae_0624_mujoco.launch.py` 新增
  - `ai_sapiens_description/` 0624 URDF/MJCF/meshes 新增
  - `deps_onnxruntime/`（x86）与 `deps_onnxruntime_aarch64/`（arm64 wheel 提取）新增
  - `docker/` 交叉编译工具（toolchain*.cmake、Dockerfile.aarch64、install_cross_toolchain.sh 等）新增
  - `docs/`（modelae_0624_deploy_plan.md）新增
- **本次会话目标任务**：
  1. 用 qemu-aarch64-static 冒烟验证交叉编译的 ONNX 推理链路（sim2real_node 的 policy 推理底座）
  2. 生成真机 harness 的通信协议适配方案（不能凭空捏造数据）
  3. 参考 0624 并联雅可比解算，把腰/踝并联解算写进 x86 与 arm64 两个 sim2real 系统，双架构交叉编译冒烟

---

## 1. 任务一：qemu 冒烟测试 sim2real_node

### 1.1 背景
在 Jetson Thor 移植前验证 aarch64 交叉编译的 ONNX 推理可用。
复用 `/tmp/test_ort_infer_aarch64.cpp`（384 维观测 → 29 维动作，API 与 `onnx_inference.cpp` 一致）
与交叉编译产物 `/tmp/test_ort_infer_sys`（aarch64 ELF，工具链 `/home/anton/aarch64-cross` GCC 11.4 免 root 解压版）。

### 1.2 复现错误
```bash
qemu-aarch64-static -L /home/anton/sysroot-arm64 \
  -E LD_LIBRARY_PATH=/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu:/opt/ros/jazzy/lib \
  /tmp/test_ort_infer_sys
```
输出：
```
model loaded OK, input=Te
U output=Te
U
FAIL: Invalid input name: @U
exit=1
```
特征：`input`/`output` 打印出乱码（带换行），`Run` 报 Invalid input name。

### 1.3 根因分析（重点）
x86 上同一程序通过（结果正常），qemu 下失败 → 典型的**未定义行为在不同分配器下表现不同**。

问题代码（/tmp/test_ort_infer_aarch64.cpp）：
```cpp
char * in_name = session.GetInputNameAllocated(0, alloc).get();   // ← 悬垂指针
char * out_name = session.GetOutputNameAllocated(0, alloc).get(); // ← 悬垂指针
```
`GetInputNameAllocated()` 返回 `Ort::AllocatedStringPtr`（**临时对象**），其析构会释放底层字符串。
`get()` 拿到的裸指针在语句结束后即悬垂；第二条语句分配 `out_name` 时复用同一块内存，
导致 `in_name` 指向被改写/越界的垃圾（出现 `Te\nU` 与 `@U` 前缀正是 freed-memory 复用特征）。
x86 glibc allocator 恰好未立即复用该块，掩盖了 bug。

**与部署代码对比**：`onnx_inference.cpp` 正确处理——局部变量 `auto input_name = session_->GetInputNameAllocated(...)`
存活到语句块结束，`emplace_back(input_name.get())` 在其存活期内完成 std::string 拷贝，安全。

### 1.4 修复
保留 `AllocatedStringPtr` 生命周期后再取裸指针：
```cpp
auto in_name_alloc = session.GetInputNameAllocated(0, alloc);
auto out_name_alloc = session.GetOutputNameAllocated(0, alloc);
const char * in_name  = in_name_alloc.get();
const char * out_name = out_name_alloc.get();
```

### 1.5 验证结果
```bash
onnxruntime cpuid_info warning: Unknown CPU vendor. cpuinfo_vendor value: 0
model loaded OK, input=obs output=actions
inference OK: 29 elems, first=-5.963583 last=-0.266337
exit=0
```
**结论**：policy.onnx（输入 `obs` 384 维 / 输出 `actions` 29 维）在 aarch64/onnxruntime 上可完整加载推理，sim2real_node 的 ONNX 链路在 Thor 上可行。

### 1.6 涉及文件（tmp，非仓库）
| 文件 | 状态 | 说明 |
|---|---|---|
| /tmp/test_ort_infer_aarch64.cpp | 修改 | 修复 AllocatedStringPtr 悬垂指针 |
| /tmp/test_ort_infer_sys | 重建 | 交叉重编（aarch64-cross GCC 11.4 + deps_onnxruntime_aarch64） |

---

## 2. 任务二：真机 harness 通信协议适配方案

### 2.1 调研（只读，未改代码）
- `Model_A_E/hfzhao/bot_main/`：真机侧 C++ 主程序，含 `actuator_module`
  （protocol：天工 `tenkun_canfd` 0x11/0x80 帧；transport：SOEM EtherCAT + `tenkun_gateway`；
  core：`MotorCommand{pos,vel,ff,kp,kd}` FPMIX）；
  路由表在 `evt11.yaml`（29 轴 CAN ID / 从站 / 通道）。约束：网关当前单模组、PDO 固件 320B 仅 12 槽。
- `ai_sapiens_sim2real/`：ros2_control `SystemInterface` 插件体系（`MujocoSystem` 为 sim 实现），
  输出 `JointImpedanceCommand{positions,feedforward,kp,kd}`（力位混合），与 `MotorCommand` 天然对齐。
  数据流：joint_states/imu → 策略 → `joint_group_impedance_controller/commands` → 硬件 write()。
  结论：只替换硬件插件即可，策略/观测/状态机零改动。

### 2.2 关键决策（用户确认，AskUserQuestion 3 项）
| 决策点 | 选择 | 依据 |
|---|---|---|
| 总线形态 | EtherCAT 网关 + 天工 CAN FD（EVT11/TenkunGateway） | 上表既定拓扑 |
| 实现路线 | ros2_control 插件 + 复用 bot_main actuator_module | 改动最小 |
| 频率 | 500Hz 下发 + 1kHz EtherCAT worker + 100ms 看门狗 | bot_main 实测频率 |

### 2.3 产出
**`docs/modelae_0624_real_harness_protocol.md`**（新文件，方案全文）：
- 架构：RealRobotSystem 插件 → BusManager(TENKUN_GATEWAY) → TenkunGateway → SOEM → 3 网关 → CAN FD
- 协议映射表：JointImpedanceCommand → MotorCommand（vel 置 0 建议）→ 0x11 帧字段（Kp/Kd uint16 BE、
  pos 度 float32 BE、vel rpm、ff int16、seq）
- 0x80 反馈 → MotorState（含电流×Kt 换算力矩、温度）
- 29 轴完整路由表（全局 ID/关节/网关 index/SOEM 从站/通道/CAN ID）
- 使能序列 0x10 0x00 → 0x13 → 0x20 0x00 → 0x10 0x01 → 0x17 → 保持 0x11
- 故障语义：100ms 超时/TX 溢出/20 周期 WKC 异常 → 锁存失能（重启恢复）
- P0 前置项：网关多模组化、腿板 14 槽固件确认、实机拓扑核对
- P1：Kt/额定参数标定、零偏/方向、腰踝并联解算、kp/kd 增益迁移
- 所有数值来自代码/配置原文，未标定项明确标注"待实机确认"（未捏造）

### 2.4 回退
无。方案阶段未写任何代码，无回退。

---

## 3. 任务三：并联解算模块移植（x86 + arm64 双架构）

### 3.1 参考源码调研（只读）
- `bot_main/src/node/kinematics/module/controller_module/method/pm_leg_pd.{h,cc}`（PMLegPD）：
  踝并联 FK/IK/雅可比（Newton + 有限差分雅可比），几何参数从配置读（PmAnkle），当前仅踝。
- `Model_A_E/29Dof_stardynamics_humaoid/common/src/Controllers/CloseChainMapping.{h,cpp}`（更强）：
  腰 + 踝通用并联（`makeWaistParallelParams` / `makeAnkleParallelParams` 硬编码 XJDL 几何），
  闭式 IK（二次消元 + 分支选择）、解析雅可比（空间速度法）、Newton FK、`Decouple` 封装。
- **几何参数**：`evt11.yaml` / `dvt10.yaml` 的 PmAnkle 均 `enabled: false` 且无几何值；
  唯一完整参考几何在 CloseChainMapping.cpp（XJDL `xjdl_description/*_ankle_parallel_kinematics_xjdl.xml`）。
  → 决策：**代码完全参数化，几何采用 XJDL 参考值并明确标注"0624 实机需标定"**（不捏造）。
- **0624 URDF**：腰/踝为串联 DOF（训练/策略语义），踝 pitch[-0.908,0.524] roll±0.349、腰 roll±0.035/pitch±0.052
  → 解算是"URDF DOF 语义 ↔ 实机并联电机角"的桥接，插在硬件接口层 write(IK)/read(FK)；
  策略观测、JointCommandPublisher、ObservationManager 全部零改动。

### 3.2 代码移植（新建文件）
1. `ai_sapiens_sim2real/include/ai_sapiens_sim2real/parallel/parallel_kinematics.hpp`
   - `ParallelKinematicsParams`（rod_len / r_A / r_B0 / r_C_body / r_pitch_joint_in_ref /
     r_roll_in_pitch / motor_abs_limit_rad / joint_dir）
   - `make_ankle_params(bool left_leg)` / `make_waist_params()`（XJDL 参考值，注释注明待标定）
   - `ParallelMechanism`：`ik(q_pitch,q_roll,theta_hint)` / `fk(theta_0,theta_1,hint_q)` /
     `joint_velocity` / `joint_torque`
   - `IkResult` / `FkResult`（含 failure_code；kFk* 常量：JacobianNan / MotorLimit / NoConvergence / BadGeometry）
   - 依赖仅 Eigen3，无 ROS。
2. `ai_sapiens_sim2real/src/parallel/parallel_kinematics.cpp`
   - 数值内核迁移自 CloseChainMapping：`single_chain_theta`（闭式 IK）、`compute_jacobian`（解析雅可比）、
     Newton FK（`newton_step` + LM 退化 + 步长限制 0.4rad）
   - **移植修正 ①（重要）**：FK Newton 迭代内的 IK 求值必须传入"目标电机角 θ_ref"作为分支 hint
     （`ik(x, theta_ref.data())`），保证迭代始终与目标同分支；原实现传 nullptr 导致大步长/网格扫描时
     误入另一个物理分支而显著失败。（对应 CloseChainMapping 的做法：`ParallelInverseKinematics(..., &theta_ref_internal)`）
3. `ai_sapiens_sim2real/test/parallel_kinematics_smoke.cpp`（冒烟自检，普通 main()，无 gtest）
   - 对左踝/右踝/腰三组几何：**电机空间扫描**（θ 网格）→ FK(hint=上帧 q) → IK 还原 θ，双向自洽
   - q 域可达性探测 + 速度/力矩映射 sanity
   - PASS 判定：双向自洽 100% 且 fk_fail < fk_ok 且 q 可达率 > 20%（不可达 θ 组合被正确拒绝是合格行为）
4. `ai_sapiens_sim2real/CMakeLists.txt`
   - 新增 `parallel_kinematics_core`（STATIC，链接 Eigen3::Eigen）
   - 新增 `parallel_kinematics_smoke` 可执行（链 core + Eigen）
   - `ai_sapiens_sim2real_node` 链接 `parallel_kinematics_core`
   - install 加入 core 与 smoke

### 3.3 冒烟测试迭代记录（含"回退/修正"全过程）

| 迭代 | 改动 | 结果 | 处置 |
|---|---|---|---|
| 0 | 初版移植 + q 域网格扫描 | 踝 95%/92%、腰 81% 通过——边缘点 θ≈±π/2 奇异带 + 网格大步长跨不可达带，大量 fk 失败 | 未通过 → 迭代 1 |
| 1 | FK 迭代 IK 传 θ_ref 作 hint（移植修正①） | 通过率提升，但 q 域扫描边缘仍失败（θ 组合物理不可达） | 判定标准不合理 → 迭代 2 |
| 2 | **验证方案回退重设计**：q 域扫描 → **电机空间扫描**（θ→FK→IK→θ'），θ 域 ±1.4（踝）/±1.0（腰）避开 90° 奇异带 | fk_ok 内双向自洽 **100%**（左踝 191/191、右踝 192/192、腰 103/103）；fk_fail 34/33/18 均为物理不可达 θ 组合（code -2/-4），属正确拒绝 | PASS 标准改为：自洽 100% && fk_fail<fk_ok && q 可达>20% → **ALL PASS** |
| 3 | warning 清理（代码级整理，无功能变化）：删除未使用的 `geometry_ok`；删除 `compute_jacobian` 未用参数 `chain`；合并 `single_chain_theta` 冗余 `r_C` 变量；`(void)q_roll` 标注 | `-Wall -Wextra` 零警告，x86 直编仍 ALL PASS | 通过 |

x86 直编（快速验证用，不含 ROS）：
```bash
g++ -std=c++17 -O2 -Wall -Wextra -I include -I /usr/include/eigen3 \
    -o /tmp/pk_smoke_x86 test/parallel_kinematics_smoke.cpp src/parallel/parallel_kinematics.cpp
```

### 3.4 x86 正式构建（colcon）
**阻塞 ①：宿主 `/opt/ros/jazzy` 被 arm64 库污染**（历史交叉构建 mount 覆盖残留：
`libstd_msgs__rosidl_generator_c.so` 等大量 .so 为 aarch64 ELF；dpkg 未记录任何 ros-jazzy 包，
无法用 apt reinstall 恢复）。
→ **绕行（环境级变更，非代码回退）**：x86 构建改在容器 `ai-sapiens:x86_64-gc13`（干净 x86 ROS）内执行。

**阻塞 ②：容器内 ONNX 头不完整**：容器 `/usr/local/include/onnxruntime` 缺
`onnxruntime_cxx_inline.h`（find_path 命中了 PATHS 而未命中 deps HINTS）。
→ 修复：run 时挂载 deps 头到 `/usr/local/include/onnxruntime`（`-v .../deps_onnxruntime/include/onnxruntime:/usr/local/include/onnxruntime`）
并删容器内旧头。

构建（隔离输出目录，避免与宿主编译残留冲突）：
```bash
docker run --rm -v /home/anton/RobotDisk/_research/aisapiens/ai_sapiens:/workspace/ai_sapiens \
  -v .../deps_onnxruntime/include/onnxruntime:/usr/local/include/onnxruntime \
  -w /workspace/ai_sapiens ai-sapiens:x86_64-gc13 bash -lc \
  "source /opt/ros/jazzy/setup.bash && colcon build \
     --packages-select ai_sapiens_interfaces ai_sapiens_sim2real \
     --build-base xbuild_x86/build --install-base xbuild_x86/install \
     --cmake-args -DBUILD_TESTING=OFF -DONNXRUNTIME_ROOT=/workspace/ai_sapiens/deps_onnxruntime"
```
宿主残留的半成品 `build/install/log` 已尝试清理：`build/`、`install/` 删除成功；
`log/` 内文件为容器 root 属主，普通用户无法删除（遗留项，见 §5）。

x86 产物冒烟：
```bash
/home/anton/RobotDisk/_research/aisapiens/ai_sapiens/xbuild_x86/install/.../parallel_kinematics_smoke
# RESULT: ALL PASS  (exit=0)
```

### 3.5 arm64 交叉构建 + qemu 冒烟
**阻塞：rosidl_generator_rs 在交叉环境失败**（`em.py` AttributeError，`ai_sapiens_interfaces__rs` 无法生成）。
→ **环境级回退：禁用 rust 生成器**（只需 C++ 绑定）：
```bash
rm -rf /opt/ros/jazzy/share/ament_index/resource_index/rosidl_generator_packages/rosidl_generator_rs \
       /opt/ros/jazzy/share/rosidl_generator_rs     # 容器内, --rm 一次性
```
（注意：仅删 share 目录不够，还必须删 ament index 条目，否则生成的构建脚本仍引用 rs target。）

构建：
```bash
docker run --rm -v ai_sapiens:/workspace/ai_sapiens \
  -v /home/anton/sysroot-arm64:/sysroot-arm64 -v /tmp:/tmp \
  -e ROS_CROSS_SKIP_PYTHON=1 -w /workspace/ai_sapiens ai-sapiens:x86_64-gc13 bash -lc \
  "source /opt/ros/jazzy/setup.bash && colcon build \
     --packages-select ai_sapiens_interfaces ai_sapiens_sim2real \
     --build-base /tmp/xbuild_arm64/build --install-base /tmp/xbuild_arm64/install \
     --cmake-args -DCMAKE_TOOLCHAIN_FILE=/workspace/ai_sapiens/docker/toolchain-aarch64-container.cmake \
                  -DBUILD_TESTING=OFF -DONNXRUNTIME_ROOT=/workspace/ai_sapiens/deps_onnxruntime_aarch64"
```
qemu 冒烟：
```bash
qemu-aarch64-static -L /home/anton/sysroot-arm64 \
  -E LD_LIBRARY_PATH=/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu \
  /tmp/xbuild_arm64/install/.../parallel_kinematics_smoke
# RESULT: ALL PASS  (exit=0)
```

### 3.6 双架构验证矩阵

| 架构 | 构建 | 冒烟 | smoke 关键指标 |
|---|---|---|---|
| x86 | 容器 colcon，`xbuild_x86/install` | 本机直跑 | 踝双向自洽 100%、q 可达 100%；腰 100%/70.4%；RESULT: ALL PASS |
| arm64 | toolchain-aarch64-container 交叉 | qemu-aarch64-static | 同上，RESULT: ALL PASS |

---

## 4. 回退与清理记录（汇总）

| 项 | 类型 | 内容 | 结果 |
|---|---|---|---|
| 冒烟验证方案 | 方案回退 | 最初 q 域网格扫描（fails 多）→ 改为电机空间双向自洽扫描 | 通过 |
| FK 迭代 hint | Bug 修正 | Newton 内 IK 传 θ_ref 分支 hint（移植修正①） | 通过 |
| 死代码清理 | 代码整理 | 删 `geometry_ok`、`compute_jacobian::chain` 参数、合并冗余 `r_C` | 零警告 |
| x86 构建环境 | 绕行 | 宿主 ROS 被 arm64 污染 → 容器内构建（隔离输出 xbuild_x86） | 通过 |
| onnx 头缺失 | 修复 | 挂载 deps 头到容器 /usr/local/include/onnxruntime | 通过 |
| rust 生成器 | 环境回退 | 交叉构建临时禁用 rosidl_generator_rs（容器一次性） | 通过 |
| 宿主残留目录 | 未完成清理 | build/install 已删；log/ 因容器 root 属主无法删（需 sudo） | 遗留 |
| ONNX 输入名悬垂指针 | Bug 修正 | 任务一已修复（tmp 测试程序） | 通过 |

## 5. 遗留事项

1. **宿主 `/opt/ros/jazzy` 大范围被 arm64 库污染**（sim2real 全部 .so 级依赖）。当前 x86 开发/构建须走容器；
   若要恢复宿主 x86 ROS，需从 ROS/Ubuntu 源重装整套 ros-jazzy 包（无 dpkg 记录，代价较大，建议另开任务）。
2. **并联几何参数为 XJDL 参考值**（`make_ankle_params` / `make_waist_params` 内硬编码），
   0624 实机腰/踝安装尺寸未标定前不得当真值；上机前必须替换并复测 smoke。
3. **宿主 `ai_sapiens/log/` 存在容器 root 属主文件**，普通用户无法删除（不影响构建，可选 sudo 清理）。
4. **并联解算尚未接入硬件插件**：写端 IK / 读端 FK 需在 RealRobotSystem（P0 待建）的
   write()/read() 中按 6 DOF 并联组索引（踝 4/5、10/11、腰 13/14）调用 `ParallelMechanism`。
5. arm64 交叉构建需在容器运行前删除 rust 生成器 index 条目（`--rm` 容器每次重建，命令中已内置）。