# 0624 真机 Harness 通信协议适配方案（Jetson Thor）

> 目标：把 `ai_sapiens_sim2real`（ROS2 + ros2_control）的力位混合阻抗命令下发到 0624 真机电机，
> 并把关节状态回读给策略，实现 sim2sim -> sim2real 全链路复用。
> 本方案基于 `Model_A_E/29Dof_stardynamics_humaoid`（真机控制原型, 通信/PD/遥控来源）、
> `hfzhao/bot_main`（CiH408 CAN FD 帧格式同源参考）与 `ai_sapiens_sim2real` 的源码/配置核对，
> 不包含未经实机确认的参数。策略以 **0624（384 维 4 帧×96, 速度限用 modelae_0624_config）为准**。

---

## 1. 背景与目标

### 1.1 两侧接口天然对齐

| 侧 | 命令结构 | 坐标/单位 | 位置 |
|---|---|---|---|
| 策略侧 `JointImpedanceCommand` | `positions/feedforward/kp/kd` | rad / Nm / Nm·rad^-1 / Nm·(rad/s)^-1 | [JointImpedanceCommand.msg](../../ai_sapiens_interfaces/msg/JointImpedanceCommand.msg) |
| 真机侧 `MotorCommand` | `target_position/target_velocity/feedforward_torque/kp/kd` | rad / rad·s^-1 / Nm / - / - | [types.h](../../../Model_A_E/hfzhao/bot_main/src/node/kinematics/module/actuator_module/core/types.h) |

两边都是 **FPMIX 力位混合**：PD 由电机模组内部执行，上层只下发 `pos + ff + kp + kd`。
策略侧命令可直接映射，无需改变 RL 策略或观测组装。

### 1.2 已确认的选型（用户决策）

1. **总线形态**：EtherCAT 网关 + 天工 CAN FD（对应 `TENKUN_GATEWAY` / EVT11 形态），
   一张网卡（旧 RTI 为 `enp4s0`）通过 SOEM 挂 3 个网关从站，板内转天工 CAN FD。
2. **实现路线**：新建 ros2_control `SystemInterface` 插件替换 `MujocoSystem`，
   **复用 bot_main 的 `actuator_module`（protocol + transport）**，策略节点零改动。
3. **频率**：控制下发 500 Hz，harness 内部 EtherCAT worker 1 kHz。

---

## 2. 部署架构

```
[ai_sapiens_sim2real_node 1000Hz RT 线程, 与仿真完全相同]
   sense:  /joint_states + imu_sensor_broadcaster/imu
   decide: ModeController (Damping/ReadyPose/Velocity 状态机)
   act:    ObservationManager -> policy.onnx (384->29)
   command: /joint_group_impedance_controller/commands  (JointImpedanceCommand)

[ros2_control_node update_rate=1000Hz]
   joint_group_impedance_controller
      |
      v
[RealRobotSystem (新插件, SystemInterface)]   <-- 只改这一层
   |  write(): 取阻抗命令 pos/ff/kp/kd, 500Hz 节流
   |  read():  回填 position/velocity/effort -> /joint_states
   |
   v
[bot_main actuator_module (复用, 静态链接)]
   BusManager(TENKUN_GATEWAY)
   -> TenkunGateway (1kHz worker)
      -> EtherCATTransport (SOEM, enp4s0, 周期 1ms)
         -> 3 个网关从站 PDO: 每通道 22B 槽位 [4B CAN ID LE][1B type][1B len][<=16B data]
            -> 天工 CAN FD: 0x11 命令 / 0x80 反馈
```

要点：
- **软件栈不换，只换 ros2_control 底部硬件插件**。策略/观测/状态机全部复用 sim 代码。
- Command/state 接口名与 Mujoco 版保持一致：`position / feedforward / proportional / derivative`。

---

## 3. 协议映射（核心）

### 3.1 JointImpedanceCommand -> MotorCommand

| JointImpedanceCommand | MotorCommand | 说明 |
|---|---|---|
| `positions[i]` | `target_position` | rad，直接透传 |
| `feedforward[i]` | `feedforward_torque` | Nm，直接透传 |
| `kp[i]` | `kp` | 直接透传 |
| `kd[i]` | `kd` | 直接透传 |
| —— | `target_velocity = 0` | **建议**：PD 已由电机内部闭环，速度字段置 0，避免与 kd 双重作用 |
| `dof 限位` | 软限位 | 采用 `ControllerModule.dof_limit_min/max + error`（evt11.yaml 已有 29 值） |

### 3.2 MotorCommand -> 天工 CAN FD `0x11` 混合命令帧（16 byte，大端）

`EncodeCommand` 帧格式见 [tenkun_cih408.h](../../../Model_A_E/29Dof_stardynamics_humaoid/third-party/tenkun-soem/src/tenkun/tenkun_cih408.h)
（bot_main `tenkun_canfd.cc` 为同款 CiH408 帧实现，可互为核对）。

| 字节 | 字段 | 编码 | 范围 |
|---|---|---|---|
| 0 | 头 `0x11` | 混合命令 | —— |
| 1-2 | Kp | uint16 大端 | clamp [0, 5000]，`(uint16)(kp + 0.5)` |
| 3-4 | Kd | uint16 大端 | clamp [0, 500] |
| 5-8 | 目标位置 | float32 大端 = rad * 180/pi | clamp [-12.5, 12.5] deg |
| 9-12 | 目标速度 | float32 大端 = rad/s * 30/pi | clamp [-18, 18] rpm |
| 13-14 | 前馈力矩 | int16 大端 | clamp [-min(max_torque,200), +min(max_torque,200)] Nm |
| 15 | 序列号 | uint8 递增 | 每命令 +1 |

### 3.3 CAN FD `0x80` 反馈帧（16 byte）-> MotorState

| 字节 | 字段 | 解析 | 注 |
|---|---|---|---|
| 1-2 | 模式/错误码 | `error_code = (b1<<8|b2) & 0xfff`，`status_word = >>12` | 非 0 即锁存失能 |
| 3-6 | 位置 | float32 大端 deg -> rad | **用于 /joint_states** |
| 7-10 | 速度 | float32 大端 rpm -> rad/s | |
| 11-12 | 电流 | int16/100 A，`torque = A * Kt`（Kt 见 §8） | |
| 13 / 14 | 电机 / MOS 温度 | `byte - 50` ℃ | 监控用 |

特殊帧：
- 使能 ACK 2B `0x80`：不含位置，跳过。
- 失能/空闲 `{0x10, 0x00}`：抢占式清空待发队列（TenkunGateway.Send 内实现）。

### 3.4 网关槽位（TenkunGateway，1 kHz worker）

- PDO 布局：`8B 头 + N * 22B`；`ChannelCount(320B)=12, (330B|400B)=14`。
- 本机器人**腿板最多用 12 通道**（§4 路由，chan 1-12）——320B/12 槽固件足够，14 槽仅留余量。
- 每通道 22B：`[CAN ID uint32 LE][type=0 标准帧][len<=16][CAN 数据]`。
- **每 EtherCAT 周期刷新同槽 `last_payload`**（不清空），否则 500 Hz Send 间隔会丢 CAN FD 帧。
- RX 校验：ID 匹配 + `(type & 0x03)==0` + len 1..16，否则忽略（并限频告警）。

---

## 4. 关节-路由表（29 轴，真机拓扑 = 29Dof `rti_whole_body.json`）

`global_motor_ids` 即策略观测/命令的数组索引，与 URDF 关节顺序一致：
左腿 6 -> 右腿 6 -> 腰 yaw/roll/pitch -> 左臂 7 -> 右臂 7。
**3 个 EtherCAT 从站**：index 2=腿板（12 通道 1-12）、index 0=左臂板（7）、index 1=腰+右臂板（10）。
`gateway_slave_index`(0..2) + 1 = SOEM 从站号；通道从 1 开始。

| 全局 ID | URDF 关节 | 网关 index | SOEM 从站(index+1) | 通道 | CAN ID |
|---|---|---|---|---|---|
| 0 | left_hip_pitch_joint | 2 | 3 | 1 | 81 |
| 1 | left_hip_roll_joint | 2 | 3 | 2 | 82 |
| 2 | left_hip_yaw_joint | 2 | 3 | 3 | 83 |
| 3 | left_knee_joint | 2 | 3 | 4 | 84 |
| 4 | left_ankle_pitch_joint | 2 | 3 | 5 | 85 |
| 5 | left_ankle_roll_joint | 2 | 3 | 6 | 86 |
| 6 | right_hip_pitch_joint | 2 | 3 | 7 | 97 |
| 7 | right_hip_roll_joint | 2 | 3 | 8 | 98 |
| 8 | right_hip_yaw_joint | 2 | 3 | 9 | 99 |
| 9 | right_knee_joint | 2 | 3 | 10 | 100 |
| 10 | right_ankle_pitch_joint | 2 | 3 | 11 | 101 |
| 11 | right_ankle_roll_joint | 2 | 3 | 12 | 102 |
| 12 | waist_yaw_joint | 1 | 2 | 10 | 51 |
| 13 | waist_roll_joint | 1 | 2 | 8 | 49 |
| 14 | waist_pitch_joint | 1 | 2 | 9 | 50 |
| 15 | left_shoulder_pitch_joint | 0 | 1 | 1 | 17 |
| 16 | left_shoulder_roll_joint | 0 | 1 | 2 | 18 |
| 17 | left_shoulder_yaw_joint | 0 | 1 | 3 | 19 |
| 18 | left_elbow_joint | 0 | 1 | 4 | 20 |
| 19 | left_wrist_roll_joint | 0 | 1 | 5 | 21 |
| 20 | left_wrist_pitch_joint | 0 | 1 | 6 | 22 |
| 21 | left_wrist_yaw_joint | 0 | 1 | 7 | 23 |
| 22 | right_shoulder_pitch_joint | 1 | 2 | 1 | 33 |
| 23 | right_shoulder_roll_joint | 1 | 2 | 2 | 34 |
| 24 | right_shoulder_yaw_joint | 1 | 2 | 3 | 35 |
| 25 | right_elbow_joint | 1 | 2 | 4 | 36 |
| 26 | right_wrist_roll_joint | 1 | 2 | 5 | 37 |
| 27 | right_wrist_pitch_joint | 1 | 2 | 6 | 38 |
| 28 | right_wrist_yaw_joint | 1 | 2 | 7 | 39 |

**腿板通道 1-12 连续**（右踝 roll 在通道 12）——320B/12 槽固件即可承载，
不再需要 13/14 槽（此前 bot_main evt11.yaml 的 13 通道布局与真机原型不一致，弃用）。

电机：**TenKun CiH408 系列**（CAN FD 仲裁 1Mbps / 数据 5Mbps，大端），
29 路均为 CAN 逻辑通道经网关接线；**注意**：腰 roll/pitch、踝 pitch/roll 为
并联电机槽位（每 2 电机驱动 2 DOF），上表是 URDF 自由度 -> 电机槽位映射，
解算见 §8.5/§10（几何待标定）。

---

## 5. 初始化 / 使能序列（天工）

`GetInitSequenceWithDuration`（10 帧，全部寻址，总约 240ms）：

```
1. {0x10, 0x00}   idle           40ms
2. {0x13, 0x3f, ...}  参数帧      40ms
3. {0x20, 0x00}   停止            40ms
4. {0x10, 0x01}   使能            40ms
5. {0x17}         清除故障        40ms
6. EncodeCommand(MotorCommand{})   40ms   # 保持第一帧 0x11
```

就绪判定 `IsReady`：`!fault_latched && error_code==0 && 收到过带位置的 0x80`。
任何一步 0x80 带错误码 -> 锁存失能 -> 停止并使能失败退出（不自动重试）。

---

## 6. 安全与故障语义（与 bot_main 一致）

| 故障 | 触发条件 | 动作 |
|---|---|---|
| 命令超时 | 100 ms 无新命令（`now - last_command > 100ms`） | `fault_latched -> 固定发失能帧` |
| TX 队列溢出 | pending > 16 | 同上 |
| WKC 异常 | 连续 20 个 EtherCAT 周期 `WKC < expected` | 同上 |
| 协议错误 / NaN | kp/kd 负、非有限值、反馈非有限 | `EncodeCommand -> idle` |
| 状态字非就绪 / 模式丢失 | 反馈 error_code != 0 | 同上 |

**锁存失能后无自动复位，必须重启应用重建驱动**。策略侧须实现：任何异常 -> 切换
Damping/ReadyPose 模式并让出总线，绝不带病下发位置命令。

---

## 7. 频率与时序设计

```
ros2_control      1000 Hz   update_rate 保持, controller 不感知频率差异
RealRobotSystem.write()    1000 Hz 被调 -> 内部 500 Hz 节流:
                           if (now - last_publish >= 2ms) 下发 29 轴 0x11
TenkunGateway worker        1 kHz   每周期 ClearOutputs -> 刷新每槽 last_payload -> UpdateCyclic
天工电机侧                  FPMIX 500 Hz 命令率, PD 内部 解算
```

- 500 Hz 下发 + 1 kHz worker 留有 2x 冗余刷新，规避丢帧。
- 反馈读取：`read()` 非阻塞 `ReceiveBatch(0, N)` 消化网关 RX 队列。

---

## 8. 电机参数（来源 29Dof `XjdlFieldBusHost.cpp` + `mujoco_simulation.json`，实机需核对）

- 电机为 **TenKun CiH408**（非 bot_main motor_database 的 HRA* 型号 list）。
- **Kt（电流->力矩）**：29Dof 硬编码 `kMotorCurrentToTorqueNmPerA`（`XjdlFieldBusHost.cpp:60-66`）：
  髋 pitch/roll、膝 `2.1`；髋 yaw、腰 yaw `2.436`；踝 `2.6`；肩、肘 `2.7`；腰 roll/pitch、腕 `2.34`。
- **力矩软限（harness clamp 参考）**：`mujoco_simulation.json` 的 `joint_tau_limit`（29 序）：
  腿 `200,120,120,200,50,50`（×2）；腰 `120,25,25`；臂 `40×7, 20×7`（×2）。
  提示：踝/腰的 50/25 N·m 与宿主二次 PD clamp（§8.5）量级一致，可作 FF 限幅与软限。
- 电流->力矩：`torque = current_A * Kt`；FF 限幅沿用 `min(max_torque, 200)` 协议侧保护。

---

## 8.5 真机 PD 方案（来自 29Dof `XjdlFieldBusHost.cpp::RunLegMotor`，直接沿用）

真机 PD 是**两段式**，与仿真"单一 affine actuator PD"不同，harness 必须实现第二段：

```
第 1 段 (策略/FSM 层, ai_sapiens 侧): BehaviorOutput -> JointImpedanceCommand
     positions/feedforward(≈0)/kp/kd  -- 与仿真同语义, 直接映射 MotorCommand
第 2 段 (真机宿主侧, harness 内实现): 仅 踝(4,5,10,11)/腰(13,14) 附加 PD
```

**第 2 段规则（29Dof 已验证逻辑）**：
| 规则 | 值 |
|---|---|
| 附加 PD 关节 | 踝 4,5,10,11 与 腰 13,14（并联组，FK 解出 DOF 角后计算） |
| 表达式 | `tau = kp*(qdes - q) + kd*(qddes - qd)` （q=FK 后的 DOF 角） |
| 腰增益乘子 | kp × 0.3, kd × 0.6 |
| torque clamp | 踝 ±50 N·m；腰 ±30 N·m |
| 附加 PD 后下发 | 踝 kp=0, kd=0.02；腰 kp=120, kd=12 |
| **全员缩放** | 下发前 `kp *= 10, kd *= 10`（腿×4、上肢×10/×2 的倍率旧代码已注释, 只用统一 ×10）|

> 说明：上表 `kp*10/kd*10` 与踝 kp=0 是 CiH408 固件阻抗单位与宿主二次 PD 的既定配合，
> 已随真机原型验证，harness 应**逐字沿用**，不要"修正"。

**站立/阻尼增益（`robot_control_parameters.json`，2026-09-12 校对）**：
- `stand_joint_kp`（29 序）：腿 `100,100,100,150(膝),40,40` ×2；腰 `200(yaw),20,20`；臂 `40` ×14
- `stand_joint_kd`（29 序）：腿 `2,2,2,4(膝),2,2` ×2；腰 `5(yaw),2,2`；臂 `1` ×14
- `safety_damper_kd = 1.0`（阻尼安全层，全轴）
- 站位姿：`straight_stand_jpos` / `squat_stand_jpos`（ReadyPose 目标；与 ai_sapiens `default_joint_pos` 对齐时以 URDF/训练零位为准）
- **IMU 安装偏移**：`imu_offset_rpy = [0, -3.16, 0]`（约 -180.12°，真机 IMU 姿态需要该补偿后喂观测）

**RL 模式 kp/kd**：29Dof 从 ONNX metadata `joint_stiffness/joint_damping` 读（缺省 50/5）；
0624 策略由 ai_sapiens `modelae_0624_config` 的 stiffness/damping 提供——真机 harness 沿用策略输出 kp/kd 经第 2 段规则即可，无需额外表。

---

## 9. 插件实现设计（RealRobotSystem）

位置：`ai_sapiens_hardware_interfaces/real_robot_hardware_interface/`，参照
[mujoco_hardware_interface](../../ai_sapiens_hardware_interfaces/mujoco_hardware_interface) 与
[radiomaster_usb_hardware_interface](../../ai_sapiens_hardware_interfaces/radiomaster_usb_hardware_interface)（IO/重连范式）。

类：`real_robot_hardware_interface::RealRobotSystem : public hardware_interface::SystemInterface`

| 生命周期 | 职责 |
|---|---|
| `on_init` | 读参数：`ethercat_port`(=enp4s0)、`joint_names`(29)、路由表(§4: slave/channel/can_id)、Kt/max_torque(§8)、`command_rate_hz`=500 |
| `on_configure` | 构造 **29Dof 的 TenkunRti 全身调度**（3 从站 29 关节，非 bot_main 单模组 gateway）；`Open()` 校验从站 PDO 尺寸与槽位越界（腿板需 >= 12 槽）；`publish_states()` 准备 |
| `on_activate` | 执行初始化序列（§5）；全部就绪后才置 activated；任一失败 -> 失能退出 |
| `read()` | `ReceiveBatch(0,128)` -> `ProcessReceivedFrame` 填 29 轴 MotorState -> **踝/腰并联 FK（§10/parallel_kinematics）回 DOF 角** -> 回填 state_interface + 发布 /joint_states |
| `write()` | 500 Hz 节流；取 `get_command(position/feedforward/proportional/derivative)` -> 踝/腰 **IK 解算转电机角** -> `MotorCommand` -> 真机 PD 第 2 段（§8.5）+ 全员 `kp*=10/kd*=10` -> `EncodeCommand` -> `Send(can_id, 16B)` |
| `on_deactivate` | `DisableMappedMotors()` 收尾失能 -> `Close()` |

构建：
- CMake 以源码/静态库方式引入 **29Dof 的 `third-party/tenkun-soem`**（SOEM + TenkunRti +
  `tenkun_cih408` 协议编解码；XjdlFieldBusHost 的实时/IO 逻辑可拆出复用），
  依赖 SOEM；`pluginlib_export_plugin_description_file` + `PLUGINLIB_EXPORT_CLASS`。
- URDF：把 `modelae_0624_mujoco.urdf` 的 `<plugin>mujoco_hardware_interface/MujocoSystem</plugin>`
  换成新的 `<plugin>real_robot_hardware_interface/RealRobotSystem</plugin>`；
  command/state interface 声明与 Mujoco 版保持一致。

---

## 10. 工程前置项（必须，P0）

1. **tenkun_rti 全身调度接入**：29Dof 的 `tenkun_rti.hpp` 已支持 3 从站 29 关节
   （`joints_cmd_order_` 汇总裁实时序），不存在 bot_main 的"单模组总线限制"；
   P0 工作 = 将其 EtherCAT 初始化/使能/状态机/反馈解析迁入 ROS2 SystemInterface 插件
   （注意 29Dof 是裸 C++ 主循环，需要包成 `read()/write()` 实时回调）。
2. **腿板 PDO 槽数核对**：§4 拓扑腿板最多 12 通道（chan 1-12 连续），320B/12 槽固件理论上足够；
   **仍需实机确认网关固件输出 PDO >= 12 槽**，否则右踝 roll 无法路由。
3. **实机拓扑核对**：按 `rti_whole_body.json` 逐轴核对网口（enp4s0）、3 从站 index、
   每板通道布线、CAN ID 与 CAN FD 接线（参考 29Dof `tests/` 单轴探测流程）。
4. **关节方向/零偏**：CAN 报告位置与 URDF 自由度方向、CiH408 编码器零偏需逐关节标定
   （`motor_direction`、`encoder_offset`）；并联腰/踝经 FK/IK 后仍须叠加零偏语义（P1）。

---

## 11. 部署与验收路径（Thor）

```
1. 单轴网关冒烟:   tenkun_motor_test 配置单模组(global ID=0), 对指定槽位发 0x11 并看 0x80
2. 插件冒烟:       RealRobotSystem + 单轴 config, ros2_control 起, 手动 0x11 位置正弦
3. 全身挂吊:       29 轴全配置 -> 初始化/使能, KNEES_BENT 挂吊 -> Velocity 模式站立
4. Damping/ReadyPose: 状态机切 Damping 验证失能安全, ReadyPose 验证零位
5. T3 速度跟踪(小幅度): 0.5/1.0 m/s 前向 -> 录制帧对比 sim2sim(target 误差指标同 sim)
6. 全范围随机:      与训练范围一致随机指令, 320s 不摔为核心指标
```

**关键验收指标（与 sim 对齐）**：
- 使能 -> 站立过渡无跌倒（root_z 0.90+）。
- P1 0.5 m/s 跟踪误差、P4 转向误差优于 sim 基线量级。
- 全程无 `fault_latched`（100ms 看门狗不失守）。

---

## 12. 已知依赖与开放项

| 项 | 状态 | 责任 |
|---|---|---|
| tenkun_rti 全身调度迁入 ROS2 插件 | 待编码（29Dof 已有全身调度, 非单模组问题） | P0 |
| 腿板 PDO >= 12 槽实机核对 | 需实机/固件核对（§10-2） | P0 |
| Kt / max_torque 实机核对 | 29Dof 硬编码 Kt 2.1~2.7 为准, 需核对 | P1 |
| 零偏/方向标定 | 逐关节标定流程 | P1 |
| 腰/踝并联解算几何参数 | 模块已移植 (`parallel_kinematics`), 几何为 XJDL 参考值, 按 0624 实机标定 | P1 |
| 真机 PD 第 2 段 (踝/腰二次 PD + ×10) | **已明确**（§8.5, 29Dof 逻辑）, 待编码进 harness | P0 |
| IMU 安装补偿 (imu_offset_rpy=[0,-3.1,0]) | 已提供, 待真机对准 | P1 |
| 遥控桥接: 29Dof `RcControlBridge` -> ROS2 | 待编码 (P0 进入实机联调前, §13) | P0 |

---

## 13. 真机遥控对接（2026-09-12 定案, 按 29Dof `RcControlBridge` 语义）

用户端遥控器为 **Logitech USB 手柄（evdev `/dev/input/js*`），插在机载计算机（Jetson Thor）** 上；
采集/模式切换/速度映射**沿用 29Dof（真机控制原型）的 `RcControlBridge.cpp`**，不重新实现。

### 13.1 手柄链路（29Dof 侧，已存在，直接复用）

```
Logitech 手柄 (evdev /dev/input/js*, RunGamepad 100ms/10Hz)
   └─ RcControlBridge (29Dof robot/src/RcControlBridge.cpp)
        ├─ 轴映射: vx = 左摇杆X, wz = 右摇杆Y, vy=0, omega_pitch=0
        ├─ 模式键 (LB 修饰键 + 组合):
        │     LB+START -> PASSIVE(零力矩)     LB+BACK -> ESTOP(急停)
        │     LB+RB    -> LOCK_JOINT(锁关节)  LB+A  -> STAND_UP(JointPD 站立)
        │     LB+B/X/Y -> LOCOMOTION(RL 行走)
        └─ 安全: 无独立断线渐减; 依赖 FSM SafetyChecker
             (RL 状态 roll>50°/pitch>60° -> 强制 LOCK_JOINT; 初始状态即 ESStop)
```

### 13.2 桥接（新增实现项, P0）

29Dof 是**裸 C++17（非 AimRT/ROS）**，输出是自定义 `RC_mode / control_mode` 枚举；
ai_sapiens 是 ROS2（速度入口 `TwistCommandHandle` / 模式请求 / `api_heartbeat`）。
真机须新增**桥接**（推荐在 29Dof 侧加一个轻量 ROS2 publisher，或独立小节点读 RcControlBridge 状态）：

| 源 (29Dof RcControlBridge)     | 目标 (ai_sapiens ROS2)                        | 周期 |
|---|---|---|
| `v_des_x / omega_des_yaw`       | `geometry_msgs/Twist` -> `TwistCommandHandle` | 与命令周期一致(>=50Hz) |
| STAND_UP / LOCOMOTION (进入)    | 模式请求进入 ReadyPose / Velocity              | 事件 |
| PASSIVE / ESTOP / LOCK_JOINT    | 切 Damping / 急停请求                         | 事件 |
| (无)                            | API 心跳: 仍由上位机服务提供, 与手柄互斥        | - |

实现位置二选一：
- **A（推荐, 改动少）**：在 29Dof 工程增加 ROS2 publisher（其 `learning_based` 侧已链入 ROS2 消息亦可），
  直接发 Twist + 模式请求，复用其已验证的 FSM 键位逻辑；
- **B**：绕过 29Dof 手柄，用 ai_sapiens 的 joystick/`radiomaster_usb_hardware_interface` 直接读
  —— 会丢失 29Dof 已验证的 LB 组合键模式语义，不推荐。

### 13.3 对齐约束

- **速度范围以 0624 策略训练为准**：ai_sapiens `modelae_0624_config` 的速度限
  （vx[-1,2] vy[-1,1] wz[-1,1]）为**权威**（不用 29Dof 旧策略的 vx<=0.7 封顶）；
  `TwistCommandHandle` 按训练范围缩放，遥控摇杆满档语义由该缩放层统一定义。
- **策略以 0624（384 维 4 帧）为准**：观测/命令由 ai_sapiens 现有链路生成，29Dof 仅提供
  通信/PD/遥控逻辑，其 480 维旧策略不部署。
- API 上机时：手柄负责手动接管（LB 组合键），API 心跳与权限互斥仍由 ai_sapiens
  AuthorityRuntime 上升沿决定，桥接只转发，不做权限决策。

---

## 参考文件

- 真机原型（控制逻辑/通信/PD/遥控来源）：
  `Model_A_E/29Dof_stardynamics_humaoid/`
  `robot/src/XjdlFieldBusHost.cpp`（总线宿主/踝腰 PD/FK 解算点）、`robot/src/RcControlBridge.cpp`（手柄）、
  `third-party/tenkun-soem/src/tenkun/{tenkun_cih408.h, tenkun_rti.hpp}`（CiH408 帧/全身调度）、
  `config/{rti_whole_body.json, robot_control_parameters.json, rl_g1_29dof_policy.json, mujoco_simulation.json}`、
  `common/src/Controllers/CloseChainMapping.cpp`（并联 FK/IK 参考源，已移植为 `parallel_kinematics`）
- 帧格式同源参考（CiH408，可互为核对）：
  `Model_A_E/hfzhao/bot_main/src/node/kinematics/module/actuator_module/`
  `protocol/tenkun_canfd.{h,cc}`、`transport/tenkun_gateway.{h,cc}`、`core/types.h`
- 策略侧：`ai_sapiens_sim2real`（sim2real_node / control_loop / ObservationManager）、
  `ai_sapiens_hardware_interfaces/mujoco_hardware_interface`（替换模板）、
  `ai_sapiens_interfaces/msg/JointImpedanceCommand.msg`