# Model_A_E (0624wholebodyURDF) → ai_sapiens 部署蓝图与上机前测试

> 目标：把 AMP_mjlab 训练的 DVT10-0624-AMP-Flat 策略（ONNX: obs[*,384] → action[*,29]，50Hz）
> 部署到 ROBOTIS ai_sapiens 控制栈（ROS2 Jazzy + ros2_control + Zenoh），
> 分阶段完成 sim2sim 全栈验证 → 真机 harness 对接 → 上机。
> 所有数值均来自真实仓库（velocity_tracking_0624 真机参数 / AMP_mjlab 训练契约 / ai_sapiens 源码），
> 缺失处已人工确认，无编造。

---

## 1. 策略契约（来自 AMP_mjlab，已用 onnx 校验）

| 项 | 值 | 来源 |
|---|---|---|
| 模型文件 | `<run>/policy.onnx`（1.48MB 单文件，无 .data） | logs/rsl_rl/dvt10_0624_amp_locomotion/2026-09-10_03-37-18/ |
| 输入 | `obs [batch, 384]`（动态 batch），**归一化已固化进 ONNX** | runner.py `_OnnxPolicyWrapper` |
| 输出 | `actions [batch, 29]`，高斯均值，**无 tanh/clip** | rsl_rl act_inference |
| 控制频率 | 50 Hz（decimation=4 × sim dt 0.005s） | amp_env_cfg.py |
| 动作处理 | `target = raw × per-joint_scale + default_joint_pos(0) − encoder_bias(0)` | mjlab actions.py |

### obs 384 = 4 帧 × 96，time-major（**帧主序**）
```
obs = [ 帧0(96) | 帧1(96) | 帧2(96) | 帧3(96) ]   帧0=最旧, 帧3=最新
每帧 96 布局:
  [0:3]    base_ang_vel_b    机体角速度(rad/s)
  [3:6]    projected_gravity  投影重力(机体系), = quat.inv()·[0,0,-9.81]
  [6:9]    command            [vx, vy, wz] 原始值 (m/s, m/s, rad/s)
  [9:38]   joint_pos          29 关节位置（相对默认位=0）
  [38:67]  joint_vel          29 关节速度
  [67:96]  last_action       前一控制步动作(29)，初始填 0
```
> 注意：mjlab 训练用 CircularBuffer（新帧 push、旧帧淘汰），obs 时序 = [t-3, t-2, t-1, t]。
> 与默认 mjlab history_ordering="term" 不兼容 → 部署端必须按上表**手动拼 time-major**（自定义观测 term 实现）。

### 29 关节序（ONNX 输出序 = URDF 树遍历序，禁改）
```
left_hip_pitch, left_hip_roll, left_hip_yaw, left_knee, left_ankle_pitch, left_ankle_roll,
right_hip_pitch, right_hip_roll, right_hip_yaw, right_knee, right_ankle_pitch, right_ankle_roll,
waist_yaw, waist_roll, waist_pitch,
left_shoulder_pitch, left_shoulder_roll, left_shoulder_yaw, left_elbow, left_wrist_roll, left_wrist_pitch, left_wrist_yaw,
right_shoulder_pitch, right_shoulder_roll, right_shoulder_yaw, right_elbow, right_wrist_roll, right_wrist_pitch, right_wrist_yaw
```

---

## 2. 真机参数（29 关节，kp/kd 来自 velocity_tracking_0624 xjdl.py 实测；补 4 关节已人工确认）

| 关节组 | 关节 | kp | kd | effort N·m | 速限 rad/s | armature | 来源 |
|---|---|---|---|---|---|---|---|
| 髋 | hip_pitch | 180 | 18 | 235 | 10 | 0.04 | xjdl.py:71 |
| 髋 | hip_roll | 240 | 24 | 200 | 12 | 0.04 | xjdl.py:72 |
| 髋 | hip_yaw | 180 | 18 | 200 | 12 | 0.04 | xjdl.py:73 |
| 膝 | knee | 240 | 24 | 235 | 10 | 0.04 | xjdl.py:74 |
| 踝 | ankle_pitch | 60 | 6 | 50 | 4 | 0.03 | xjdl.py:105 |
| 踝 | ankle_roll | 60 | 6 | 50 | 10 | 0.03 | xjdl.py:105 |
| 腰 | waist_yaw | 60 | 6 | 120 | 12 | 0.06 | xjdl.py:130 |
| 腰 | waist_roll | 80 | 6 | 50 | 6 | 0.03 | 类推(注释块 80/6/50) |
| 腰 | waist_pitch | 80 | 6 | 50 | 6 | 0.03 | 类推(注释块 80/6/50) |
| 肩 | shoulder_pitch/roll/yaw | 20 | 2 | 40 | 10 | 0.03 | xjdl.py:163-177 |
| 肘 | elbow | 20 | 2 | 40 | 10 | 0.03 | xjdl.py:164 |
| 腕 | wrist_roll/pitch | 10 | 1 | 20 | 10 | 0.03 | xjdl.py:167-176 |
| 腕 | wrist_yaw | 10 | 1 | 20 | 10 | 0.03 | 类推(腕组 10/1/20) |

- **default_position（动作 offset）**：**不是 0** —— 以 ONNX 元数据 `default_joint_pos` 为准（= knees_bent 站立位
  `[-0.1,0,0,0.3,-0.15,0, ...]`）。训练动作 `target = raw×scale + default_joint_pos`，部署端 offset 必须等值；
  `joint_pos_rel` 观测同样减去该值。
- **ready_pose 过渡位（非 offset）**：可另配（同 xjdl init 站姿），不影响 actions.offset。
- **动作 scale**：**以 ONNX 元数据 `action_scale` 为准**（= 0.25 × effort / 训练kp，训练 kp 为 0624 XML 值 90.4/223/64.1/32.1/37.75 **不是** 真机 PD kp）。
  实测前 4 关节: hip_pitch 0.650 / hip_roll 0.263 / hip_yaw 0.415 / knee 0.263（其余 29 关节全量见 sim2real.yaml）。
  部署端不能用手写 0.25×eff/真机kp，否则动作幅值错 2~3 倍。

---

## 3. ai_sapiens 新增/修改文件清单

### 新增资产（机器人 + 策略）
| 文件 | 内容 |
|---|---|
| `ai_sapiens_description/urdf/modelae_0624/0624wholebodyURDF.urdf` | 复制自 Model_A_E/URDF/0624wholebodyURDF（含 `<mujoco>` 头），meshes 连带 |
| `ai_sapiens_description/meshes/modelae_0624/` | 0624 STL + convex（从 AMP_mjlab dvt10_0624/meshes 复制，带 7 胶囊版同源） |
| `ai_sapiens_description/mujoco/modelae_0624/scene.xml` | MuJoCo 场景（引用 0624wholebodyURDF.xml，复制自 AMP_mjlab 带胶囊版） |
| `ai_sapiens_description/ros2_control/modelae_0624/` | ros2_control 驱动（29 关节 state/command + 力矩限制） |
| `ai_sapiens_bringup/config/modelae_0624/modelae_0624_controllers.yaml` | 29 关节序 + joint_group_impedance_controller 配置 |
| `ai_sapiens_bringup/launch/modelae_0624.launch.py` / `modelae_0624_mujoco.launch.py` | 真机/仿真 launch（仿 k1，指到 0624 资产） |
| `ai_sapiens_sim2real/config/modelae_0624_config.yaml` | robot_joint_order(29) + 状态机（Damping→ReadyPose→Velocity）+ teleop |
| `ai_sapiens_sim2real/assets/modelae_0624/locomotion/velocity/walk_default/params/sim2real.yaml` | policy_joints(29) + joint_properties(kp/kd) + observations(proprio_full) + actions(scale/offset) |
| `ai_sapiens_sim2real/assets/modelae_0624/locomotion/velocity/walk_default/model.onnx` | 复制最新 policy.onnx |

### 修改代码
| 文件 | 修改 |
|---|---|
| `ai_sapiens_sim2real/src/observation/observations.cpp` | 注册新观测 term `proprio_full`：按 YAML `terms` 顺序逐项取当前值（复用已有 base_ang_vel/projected_gravity/velocity_commands/joint_pos_rel/joint_vel_rel/last_action 函数），自行维护 4 帧 deque，输出 time-major 384 维 |
| `ai_sapiens_sim2real/include/.../observations.hpp` | 声明 |
| `ai_sapiens_sim2real/src/policy/policy_runtime.cpp` | 确认输入 size 校验（384）通过；velocity 状态下 last_action 回填路径（复用现有） |

---

## 4. 对齐要求（sim2sim 与 sim2real 必须同时满足）

| 维度 | 要求 | 验证方法 |
|---|---|---|
| 控制频率 | 50 Hz 整（step_dt 0.02），与 MuJoCo sim 对齐 | log tick 间隔 |
| obs 时序 | time-major，帧序 [t-3,t-2,t-1,t]，last_action 为上一控制步 | 单步打印 obs 对比 mjlab 环境 |
| obs 数值 | 无归一化（ONNX 内置）；projected_gravity 用机体四元数逆旋 (0,0,-9.81) | 静止站立打印 obs，对准 mjlab probe |
| 关节序 | 29 关节 ONNX 序 == robot_joint_order == 控制器序 == URDF 序 | 启动打印 joint map |
| PD | kp/kd 用上表；MuJoCo 仿真同样用（hardware 内） | sim2sim PD 与 mjlab 训练 PD 行为一致 |
| 碰撞 | sim2sim 场景用带 7 胶囊 + 软碰撞模型（condim=1 body/3 foot） | 站立不抖动 |
| 动作边界 | action×scale 后 soft clip 到 0.9×joint limit | 日志统计 |

---

## 5. 测试用例清单（上机前 Checklist）

### A. 单元级（容器内，无硬件）
| # | 用例 | 通过标准 | 指标 |
|---|---|---|---|
| A1 | ONNX 加载 + 输入/输出 shape 校验 | load ok，obs[*,384]→action[*,29] | — |
| A2 | obs size 校验 | manager 报警 size==384 | — |
| A3 | 关节序一致性 | 5 处关节序 diff 为空 | — |
| A4 | sim2real.yaml 数值（kp/kd/scale/offset）与上表 diff | 全等 | — |

### B. sim2sim 行为级（MuJoCo 全栈）
| # | 用例 | 通过标准 | 指标 |
|---|---|---|---|
| B1 | 冷启动 → ReadyPose 3s → Velocity(指令 0) | 无跌倒、无 NaN | 姿态误差<10°、骨盆高 0.9±0.05m |
| B2 | vx=0.5 / 1.0 / 2.0 段速各 10s | 稳定行走 | 平均误差<0.3 m/s，无跌倒 |
| B3 | 指令 0 → vx=3.0 阶跃 | 跟得上（外推） | 达到 2.7 m/s 以上，无 div |
| B4 | vy=±0.5 / wz=±0.8 | 侧移/转向稳定 | 角速度误差<0.5 rad/s |
| B5 | 连续随机指令 60s | 全程存活 | 终止 0、slip<0.5 |
| B6 | obs 拼装对照 mjlab eval（同 seed） | ai_sapiens obs == mjlab obs（tol 1e-4） | 逐元素 diff |
| B7 | 50Hz 频率稳定 | 周期抖动<5% | 中断计数 0 |

### C. 上机前（真机 harness 对接后）
| # | 用例 | 通过标准 |
|---|---|---|
| C1 | Damping 全关节软化 3.0 | 手感正常、无抖动 |
| C2 | ReadyPose 站起（3s 过渡） | 顺利站立、姿态稳定 |
| C3 | 急停（DampingRequested）任意时刻 | <0.2s 响应 |
| C4 | Velocity 低速 0.3 m/s 起步 | 平稳起步不打滑 >1s |
| C5 | 逐步升速 0.3→0.6→1.0 | 每档稳定 5s，无异常 |
| C6 | 全程扭矩监控 | max τ < 0.8×effort 上限 |
| C7 | 关节温度/通信心跳 | 无超温、无丢包 |

### D. 跟踪指标（日志/TensorBoard，每个数据点）
- 骨盆高度（均值/σ）、俯仰角、前向速度误差、转向误差
- 关节 torque RMS / max、关节限位占用、步频、步幅、触地占比、打滑均速
- 终止数、NaN 计数、tick 周期抖动

---

## 6. 风险与待办
- [ ] 训练最终 ONNX 完成后替换（当前用联调版，契约不变）
- [ ] 真机 harness（非 Dynamixel）——首个可运行里程块待定，需要 CAN/EtherCAT 协议资料
- [ ] sim2sim 碰撞模型 = 训练模型，若真机关节摩擦/阻尼与 PD 不符，靠 kp/kd 微调窗口
- [ ] teleop 映射：训练命令上限 vx 3.0 / vy 1.0 / wz 1.57，遥控器应限幅 < 上限