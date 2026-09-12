# 0624 真机 Harness 通信协议适配方案（Jetson Thor）

> 目标：把 `ai_sapiens_sim2real`（ROS2 + ros2_control）的力位混合阻抗命令下发到 0624 真机电机，
> 并把关节状态回读给策略，实现 sim2sim -> sim2real 全链路复用。
> 本方案基于对 `Model_A_E/hfzhao/bot_main` 与 `ai_sapiens_sim2real` 的源码/配置核对，不包含未经实机确认的参数。

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

`EncodeCommand` 实现在 [tenkun_canfd.cc](../../../Model_A_E/hfzhao/bot_main/src/node/kinematics/module/actuator_module/protocol/tenkun_canfd.cc)。

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
- 每通道 22B：`[CAN ID uint32 LE][type=0 标准帧][len<=16][CAN 数据]`。
- **每 EtherCAT 周期刷新同槽 `last_payload`**（不清空），否则 500 Hz Send 间隔会丢 CAN FD 帧。
- RX 校验：ID 匹配 + `(type & 0x03)==0` + len 1..16，否则忽略（并限频告警）。

---

## 4. 关节-路由表（29 轴，来自 evt11.yaml / EVT11_HARDWARE_CONFIG.md）

`global_motor_ids` 即策略观测/命令的数组索引，与 URDF 关节顺序一致：
左腿 6 -> 右腿 6 -> 腰 yaw/roll/pitch -> 左臂 7 -> 右臂 7。

| 全局 ID | URDF 关节 | 网关 index | SOEM 从站(index+1) | 通道 | CAN ID |
|---|---|---|---|---|---|
| 0 | left_hip_pitch_joint | 2 | 3 | 1 | 81 |
| 1 | left_hip_roll_joint | 2 | 3 | 2 | 82 |
| 2 | left_hip_yaw_joint | 2 | 3 | 3 | 83 |
| 3 | left_knee_joint | 2 | 3 | 4 | 84 |
| 4 | left_ankle_pitch_joint | 2 | 3 | 5 | 85 |
| 5 | left_ankle_roll_joint | 2 | 3 | 6 | 86 |
| 6 | right_hip_pitch_joint | 2 | 3 | 8 | 97 |
| 7 | right_hip_roll_joint | 2 | 3 | 9 | 98 |
| 8 | right_hip_yaw_joint | 2 | 3 | 10 | 99 |
| 9 | right_knee_joint | 2 | 3 | 11 | 100 |
| 10 | right_ankle_pitch_joint | 2 | 3 | 12 | 101 |
| 11 | right_ankle_roll_joint | 2 | 3 | 13 | 102 |
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

电机型号（motor_database.yaml 的 `TENKUN_HRA*`）：
- 腿/髋/腰 yaw：HRA125P / HRA100P；踝：HRA55P；腰 roll/pitch：HRA60P；左臂肩 2 轴：HRA60P_PRO；其余臂/腕：HRA60P。
- **注意**：腰 roll/pitch、踝 pitch/roll 的条目是"并联电机槽位"，不是 URDF 串联自由度本身；
  上表映射的是 URDF 自由度 -> 电机槽位，真正的并联解算见 §10（P1）。

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

## 8. 电机参数（待实机标定, P1）

- `torque_constant`（Kt, Nm/A）与 `max_torque`（Nm）：`motor_database.yaml` 的 HRA 型号参数为准。
- 电流->力矩：`torque = current_A * Kt`；FF 限幅 `min(max_torque, 200)`。
- 旧 RTI 参考换算（`kMotorCurrentToTorqueNmPerA`，**仅供参考核对，需实机标定**）：
  髋 pitch/roll、膝 `2.1`；髋 yaw、腰 yaw `2.436`；踝 `2.6`；肩、肘 `2.7`；腰 roll/pitch、腕 `2.34`。
- 4226 全仿真用的 kp/kd 是"串联关节语义"，与实机逐关节增益倍率不同（旧工程对膝、右髋
  roll、上肢使用不同倍率），**不可直接当实机增益**（见 §10 P1）。

---

## 9. 插件实现设计（RealRobotSystem）

位置：`ai_sapiens_hardware_interfaces/real_robot_hardware_interface/`，参照
[mujoco_hardware_interface](../../ai_sapiens_hardware_interfaces/mujoco_hardware_interface) 与
[radiomaster_usb_hardware_interface](../../ai_sapiens_hardware_interfaces/radiomaster_usb_hardware_interface)（IO/重连范式）。

类：`real_robot_hardware_interface::RealRobotSystem : public hardware_interface::SystemInterface`

| 生命周期 | 职责 |
|---|---|
| `on_init` | 读参数：`ethercat_port`(=enp4s0)、`joint_names`(29)、路由表(slave/channel/can_id)、电机型号(Kt/max_torque)、`command_rate_hz`=500 |
| `on_configure` | 构造 `BusManager(TENKUN_GATEWAY)`；`Open()` 校验从站 PDO 尺寸与槽位越界；`publish_states()` 准备 |
| `on_activate` | 执行初始化序列（§5）；全部就绪后才置 activated；任一失败 -> 失能退出 |
| `read()` | `sim_->advance` 替换为：`ReceiveBatch(0,128)` -> `ProcessReceivedFrame` 填 29 轴 MotorState -> 回填 state_interface `position/velocity/effort` + 发布 /joint_states |
| `write()` | 500 Hz 节流；逐轴 `get_command(position/feedforward/proportional/derivative)` -> `MotorCommand` -> `EncodeCommand` -> `Send(can_id, 16B)` |
| `on_deactivate` | `DisableMappedMotors()` 收尾失能 -> `Close()` |

构建：
- CMake 以源码/静态库方式引入 `actuator_module`（protocol/transport/driver/core），
  依赖 SOEM (`libsoem`)；`pluginlib_export_plugin_description_file` + `PLUGINLIB_EXPORT_CLASS`。
- URDF：把 `modelae_0624_mujoco.urdf` 的 `<plugin>mujoco_hardware_interface/MujocoSystem</plugin>`
  换成新的 `<plugin>real_robot_hardware_interface/RealRobotSystem</plugin>`；
  command/state interface 声明与 Mujoco 版保持一致。

---

## 10. 工程前置项（必须，P0）

1. **TenkunGateway 多模组化**：当前构造器强制 `config.motors.size()==1`（单轴测试版，
   `tenkun_gateway.cc:36`），open/Run/routes 已按 Route 数组写好，需放开限制并验证 29 路由
   同时承载；`BusManager` 亦含单轴总线约束，需一并验证。
2. **腿板 PDO 槽数**：路由用腿板 13 槽（右踝 roll 通道 13），320B 固件仅 12 槽；
   **必须实机核对网关固件为 400B/14 槽（或 330B/14 槽）**，否则右踝 roll 无法路由。
3. **实机拓扑确认**：网口名（enp4s0?）、3 个从站的 index/顺序、每板 CAN FD 接线、
   通道布线必须逐轴核对（参考 `tests/dvt10/bus_test.cc`、`tests/tenkun/tenkun_test.cc` 单轴探测）。
4. **关节方向/零偏**：CAN 报告位置与 URDF 自由度方向、can fd 编码器零偏需逐关节标定
   （`encoder_offset`、`motor_direction`）。训练数据与 feedback 均按 URDF 串联自由度语义，
   实机并联腰/踝需解算（P1）。

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
| TenkunGateway 单模组限制 | 需编码放开 | P0 |
| PDO 固件 14 槽确认 | 需实机/固件核对 | P0 |
| HRA* 型号 Kt / max_torque 确认 | motor_database.yaml 为准, 实机核对 | P1 |
| 零偏/方向标定 | 逐关节标定流程 | P1 |
| 腰/踝并联解算 | 旧工程 `XjdlFieldBusHost` 迁移 | P1 |
| 实机 kp/kd 增益迁移 | 逐关节倍率换算, 不得直用仿真值 | P1 |

---

## 参考文件

- 协议/驱动：`Model_A_E/hfzhao/bot_main/src/node/kinematics/module/actuator_module/`
  `protocol/tenkun_canfd.{h,cc}`、`transport/tenkun_gateway.{h,cc}`、`transport/ethercat.{h,cc}`、
  `core/types.h`、`core/bus_manager.{h,cc}`、`driver/motor_driver.{h,cc}`
- 配置：`src/install/bin/cfg/02_robots/evt11.yaml`、`01_components/actuators/motor_database.yaml`
- 文档：`docs/DVT10_HARDWARE_CONFIG.md`、`docs/EVT11_HARDWARE_CONFIG.md`、`docs/TENKUN_SINGLE_MOTOR.md`
- 策略侧：`ai_sapiens_sim2real`（sim2real_node / control_loop / ObservationManager）、
  `ai_sapiens_hardware_interfaces/mujoco_hardware_interface`（替换模板）、
  `ai_sapiens_interfaces/msg/JointImpedanceCommand.msg`