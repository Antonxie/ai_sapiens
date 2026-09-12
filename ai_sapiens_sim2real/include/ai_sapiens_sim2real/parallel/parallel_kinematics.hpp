// Copyright 2026 ai_sapiens_sim2real contributors
//
// 真机并联机构运动学解算（腰 roll/pitch 与左右踝 pitch/roll）。
//
// 背景：
//   Model_A_E 0624/EVT11 的 URDF 使用 29 个串联自由度（训练/策略语义），
//   但实机腰 roll/pitch 与踝 pitch/roll 是"五连杆并联机构"：1 组 pitch+roll
//   由 2 个电机通过连杆（曲柄 + 连杆）驱动。上表 29 个电机槽位中,
//   成对槽位（踝内/外侧、腰两路）驱动这 3 组并联 DOF, 其余 23 个 DOF 直驱。
//
// 本模块提供：
//   - 数值内核: IK (q=[pitch,roll] -> 电机角 theta[2]), FK (电机角 -> q,
//     Newton 迭代 + 解析雅可比), 速度/力矩映射。数学内核迁移自
//     Model_A_E/29Dof_stardynamics_humaoid/common/src/Controllers/CloseChainMapping.cpp
//     (与 xjdl_description/*_ankle_parallel_kinematics_xjdl.xml 一致)。
//   - 几何参数: make_ankle_params(left/right), make_waist_params() —— 为 XJDL
//     参考值, 0624 实机装机尺寸未标定前不得视为真值 (见 make_* 注释)。
//   - ParallelMechanism: 1 组并联 (pitch/roll 两 DOF) 的高层封装。
//
// 用法 (写入 RealRobotSystem 的 write()/read()):
//   write(): 6 个并联 DOF 的策略命令 -> IK -> 2 路电机角, 替换原槽位命令;
//   read():  电机反馈角 -> FK(以上帧 q 为初值) -> DOF 角, 回填 joint_states。
// 其余 23 个 DOF 与电机 1:1 直驱, 不经过本模块。

#ifndef AI_SAPIENS_SIM2REAL__PARALLEL_KINEMATICS_HPP_
#define AI_SAPIENS_SIM2REAL__PARALLEL_KINEMATICS_HPP_

#include <Eigen/Dense>
#include <string>

namespace ai_sapiens_sim2real::parallel {

// 并联机构几何参数 (单位: 长度 mm, 角度 rad)。
// 约定: 关节向量 q = [pitch, roll]; FK 中足体上 C 点:
//   r_C = r_pitch_joint_in_ref + Ry(pitch) * (r_roll_in_pitch + Rx(roll) * r_C_body)
// pitch 轴在参考系(胫骨系)内 +Y, roll 轴在 pitch 连体系内 +X。
// 对腰等两轴共点的机构, 将 r_pitch_joint_in_ref 与 r_roll_in_pitch 置零即可。
struct ParallelKinematicsParams {
  std::string name;
  double rod_len[2]{0.25, 0.19};          // 连杆长度 (C-B), 每链一根
  Eigen::Vector3d r_A[2];                 // 电机法兰中心 (固定) [mm]
  Eigen::Vector3d r_B0[2];                // 曲柄端点零位 (theta=0 时) [mm]
  Eigen::Vector3d r_C_body[2];            // C 点在足/骨盆连体系 [mm]
  Eigen::Vector3d r_pitch_joint_in_ref{0.0, 0.0, 0.0};
  Eigen::Vector3d r_roll_in_pitch{0.0, 0.0, 0.0};
  double motor_abs_limit_rad{1.5};        // |theta| 越界视为 FK 失败
  double joint_dir[2]{1.0, 1.0};          // 关节方向 (与 motor_direction 相乘)
};

// XJDL 参考几何: 左/右踝 (足部 roll/pitch), 来自 xjdl_description
// left/right_ankle_parallel_kinematics_xjdl.xml 零位 (胫骨系)。
// 注意: 0624 实机安装尺寸尚未标定, 以下为参考值, 上机前必须按实机替换。
ParallelKinematicsParams make_ankle_params(bool left_leg);

// XJDL 参考几何: 腰 (waist_parallel_kinematics.xml), 两轴共点机构。
// 同样为参考值, 0624 实机需标定。
ParallelKinematicsParams make_waist_params();

// 逆运动学结果: 电机角 theta[2] (曲柄角, 绕参考系 +X)。
struct IkResult {
  bool ok{false};
  double theta[2]{0.0, 0.0};
};

// 正运动学结果: FK 后的关节角 q=[pitch, roll]。
struct FkResult {
  bool ok{false};
  int iterations{0};        // Newton 迭代次数 (成功时为实际次数)
  int failure_code{0};      // 失败原因, 见 kFkFailure* 常量
  double q_pitch{0.0};
  double q_roll{0.0};
};

// FK 失败码
constexpr int kFkOk = 0;
constexpr int kFkJacobianNan = -1;    // 雅可比含 NaN
constexpr int kFkMotorLimit = -2;     // 电机角超限位
constexpr int kFkNoConvergence = -3;  // Newton 未收敛
constexpr int kFkBadGeometry = -4;    // IK 闭链几何不一致 / 奇异附近

// 计算雅可比 J (2x2): J[i][j] = d(theta_i) / d(q_j), q=[pitch,roll]。
// 由解析闭式(空间速度)推导, 失败(奇异/几何无效)返回 false。
bool jacobian(
  const ParallelKinematicsParams & params,
  double q_pitch, double q_roll,
  Eigen::Matrix2d * J);

// 单组并联机构的 IK/FK 封装。
// theta_hint: 上帧电机角, 用于 IK 分支选择 (保持时间连续, 可传 nullptr 取最近分支)。
class ParallelMechanism {
 public:
  explicit ParallelMechanism(const ParallelKinematicsParams & params);

  const ParallelKinematicsParams & params() const { return params_; }

  // q=[pitch,roll] (rad, 关节语义) -> 电机角 theta[2] (rad, 曲柄角)。
  IkResult ik(double q_pitch, double q_roll, const double theta_hint[2] = nullptr) const;

  // 电机角 theta[2] -> q=[pitch,roll]。hint 用上一帧 q 加速收敛。
  FkResult fk(
    double theta_0, double theta_1,
    double hint_pitch = 0.0, double hint_roll = 0.0) const;

  // 关节角速度映射: dq/dt = J^-1 * dtheta/dt。J 为 dtheta/dq。
  bool joint_velocity(
    double q_pitch, double q_roll,
    double theta_dot_0, double theta_dot_1,
    double * q_pitch_dot, double * q_roll_dot) const;

  // 关节力矩映射: tau_q = J^T * tau_theta。
  bool joint_torque(
    double q_pitch, double q_roll,
    double tau_theta_0, double tau_theta_1,
    double * tau_q_pitch, double * tau_q_roll) const;

 private:
  ParallelKinematicsParams params_;
};

}  // namespace ai_sapiens_sim2real::parallel

#endif  // AI_SAPIENS_SIM2REAL__PARALLEL_KINEMATICS_HPP_