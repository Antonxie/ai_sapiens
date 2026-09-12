// Copyright 2026 ai_sapiens_sim2real contributors
//
// 并联机构运动学数值内核实现。
// 数学迁移自 Model_A_E/29Dof_stardynamics_humaoid/common/src/Controllers/CloseChainMapping.cpp
// (闭式 IK + Newton FK + 解析雅可比), 依赖仅 Eigen3。

#include "ai_sapiens_sim2real/parallel/parallel_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace ai_sapiens_sim2real::parallel {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kFkTolRad = 2e-8;        // FK 误差收敛阈值 (rad)
constexpr int kFkMaxIter = 50;            // FK Newton 最大迭代
constexpr double kDetJacobianEps = 1e-9;  // 雅可比奇异判定阈值
constexpr double kMaxNewtonStepRad = 0.4; // FK 单步最大步长, 防止发散
constexpr double kClampTol = 1e-10;       // IK 判别式负值容忍

// IK 闭链几何自检: 关键点间距与杆长一致 (允差 2mm)。当前 FK Newton 迭代中
// 已通过 IK 成功状态隐含校验, 暂不单独启用。

bool motor_limit_exceeded(const Eigen::Vector2d & th, double abs_limit_rad)
{
  return std::abs(th[0]) > abs_limit_rad || std::abs(th[1]) > abs_limit_rad;
}

// Newton 步: dq = J^{-1} f_err; J 奇异时退化到 LM (乘子递增)。
// 返回 false 表示奇异且 LM 也失败。
bool newton_step(
  const Eigen::Matrix2d & J_joint2motor,
  const Eigen::Vector2d & f_err,
  Eigen::Vector2d * dq)
{
  const double det = J_joint2motor.determinant();
  if (std::abs(det) > kDetJacobianEps) {
    *dq = J_joint2motor.inverse() * f_err;
    return true;
  }
  double lm = 1e-5;
  const Eigen::Matrix2d JtJ = J_joint2motor.transpose() * J_joint2motor;
  const Eigen::Vector2d rhs = J_joint2motor.transpose() * f_err;
  for (int k = 0; k < 10; ++k) {
    const Eigen::Vector2d cand = (JtJ + lm * Eigen::Matrix2d::Identity()).ldlt().solve(rhs);
    if (std::isfinite(cand[0]) && std::isfinite(cand[1]) && cand.norm() < 1e3) {
      *dq = cand;
      return true;
    }
    lm *= 4.0;
  }
  return false;
}

double normalize_angle(double a)
{
  while (a > kPi) { a -= 2.0 * kPi; }
  while (a < -kPi) { a += 2.0 * kPi; }
  return a;
}

double angle_distance(double a, double b)
{
  return std::abs(normalize_angle(a - b));
}

// 单链求解: 给定目标 q=[pitch,roll], 求该链曲柄角 theta。
// 几何: r_C 已知(旋转后), 求 r_B(theta) 使 |r_C - r_B(theta)| = rod_len。
// r_B(theta) = r_A + Rx(theta) * r_bar0。二次消元得 sin(theta) 的候选,
// 再用 theta_hint 选时间连续分支。
bool single_chain_theta(
  const ParallelKinematicsParams & params,
  int chain,
  double q_pitch, double q_roll,
  double theta_hint,
  double * theta)
{
  const double cp = std::cos(q_pitch);
  const double sp = std::sin(q_pitch);
  const double cr = std::cos(q_roll);
  const double sr = std::sin(q_roll);

  // R = Ry(pitch) * Rx(roll)
  Eigen::Matrix3d R;
  R << cp, sp * sr, sp * cr,
       0.0, cr, -sr,
       -sp, cp * sr, cp * cr;
  // r_C 世界位置 = r_pitch + R * (r_roll + r_C_body) (R 已含 Ry*Rx 复合)
  const Eigen::Vector3d r_C_world =
    params.r_pitch_joint_in_ref + R * params.r_roll_in_pitch + R * params.r_C_body[chain];

  const Eigen::Vector3d r_A = params.r_A[chain];
  const Eigen::Vector3d r_AB0 = params.r_B0[chain] - r_A;  // 曲柄零位向量
  const Eigen::Vector3d r_CA = r_A - r_C_world;

  const double rod = params.rod_len[chain];
  const double wxvx = r_CA[0] + r_AB0[0];
  const double M = rod * rod - wxvx * wxvx -
    (r_CA[1] * r_CA[1] + r_CA[2] * r_CA[2]) -
    (r_AB0[1] * r_AB0[1] + r_AB0[2] * r_AB0[2]);
  const double N = r_CA[1] * r_AB0[1] + r_CA[2] * r_AB0[2];
  const double K = r_CA[1] * r_AB0[2] - r_CA[2] * r_AB0[1];

  const double a = 4.0 * (K * K + N * N);
  const double b = -4.0 * M * K;
  const double c = M * M - 4.0 * N * N;
  const double disc_raw = b * b - 4.0 * a * c;

  if (disc_raw < -kClampTol || a < 1e-16) {
    // 目标点不可达: 机器人不能到达该 q
    return false;
  }
  const double disc = std::max(0.0, disc_raw);

  // sin(theta) 的候选 (解析二次方程)
  std::vector<double> sin_candidates;
  if (disc <= 1e-18) {
    sin_candidates.push_back(-b / (2.0 * a));
  } else {
    const double sd = std::sqrt(disc);
    sin_candidates.push_back((-b + sd) / (2.0 * a));
    sin_candidates.push_back((-b - sd) / (2.0 * a));
  }

  // 用 hint 选时间连续的物理分支 (每个 sin 有两个 theta: th 与 pi-th)
  double best_err = std::numeric_limits<double>::infinity();
  double best_dist = std::numeric_limits<double>::infinity();
  double best_theta = 0.0;
  for (double s : sin_candidates) {
    s = std::max(-1.0, std::min(1.0, s));
    const double th_asin = std::asin(s);
    const double th_candidates[2] = {th_asin, kPi - th_asin};
    for (double th_raw : th_candidates) {
      const double th = normalize_angle(th_raw);
      // 代回验证: |C - B(th)| 与杆长差异
      const double cth = std::cos(th);
      const double sth = std::sin(th);
      Eigen::Matrix3d Rx;
      Rx << 1.0, 0.0, 0.0,
            0.0, cth, sth,
            0.0, -sth, cth;
      const Eigen::Vector3d r_B = r_A + Rx * r_AB0;
      const double err = std::abs((r_C_world - r_B).norm() - rod);
      const double dist = angle_distance(th, theta_hint);
      if (err + 1e-12 < best_err ||
          (std::abs(err - best_err) <= 1e-12 && dist < best_dist)) {
        best_err = err;
        best_dist = dist;
        best_theta = th;
      }
    }
  }
  *theta = best_theta;
  // 若最优分支仍不满足约束, 说明目标不可达
  if (best_err > 1e-3) {
    return false;
  }
  return true;
}

// 解析雅可比: theta 对 q 的偏导 (空间速度法)。
// J_x (rod 约束对 pi/theta) 与 J_q (关节角速度的 pi/theta 展开) 组合。
// 两条链的约束同时构造 2x2 J, 无单链参数。
bool compute_jacobian(
  const ParallelKinematicsParams & params,
  double q_pitch, double q_roll,
  const std::vector<Eigen::Vector3d> & r_C_world,
  const std::vector<Eigen::Vector3d> & r_bar_world,
  const std::vector<Eigen::Vector3d> & r_rod_world,
  Eigen::Matrix2d * J_out)
{
  // q_roll 不出现在显式偏导中: roll 的影响由 r_rod_world(随 q 的 IK 状态) 隐式承载
  (void)q_roll;
  // 曲柄旋转轴在胫骨系 +X
  const Eigen::Vector3d s_11(-1.0, 0.0, 0.0);
  const Eigen::Vector3d s_21(-1.0, 0.0, 0.0);

  Eigen::MatrixXd J_x = Eigen::MatrixXd::Zero(2, 6);
  J_x.block<1, 3>(0, 0) = r_rod_world[0].transpose();
  J_x.block<1, 3>(1, 0) = r_rod_world[1].transpose();
  J_x.block<1, 3>(0, 3) = (r_C_world[0].cross(r_rod_world[0])).transpose();
  J_x.block<1, 3>(1, 3) = (r_C_world[1].cross(r_rod_world[1])).transpose();

  Eigen::MatrixXd J_theta = Eigen::MatrixXd::Zero(2, 2);
  J_theta(0, 0) = s_11.dot(r_bar_world[0].cross(r_rod_world[0]));
  J_theta(1, 1) = s_21.dot(r_bar_world[1].cross(r_rod_world[1]));

  // 关节空间速度: pitch 绕 +Y (过 r_pitch_joint_in_ref), roll 绕
  // R_y(pitch)*X (过 r_pitch + R_y*r_roll_in_pitch)
  const double cp = std::cos(q_pitch);
  const double sp = std::sin(q_pitch);
  Eigen::Matrix3d R_y;
  R_y << cp, 0.0, sp, 0.0, 1.0, 0.0, -sp, 0.0, cp;

  const Eigen::Vector3d omega_pitch(0.0, 1.0, 0.0);
  const Eigen::Vector3d v_pitch = omega_pitch.cross(-params.r_pitch_joint_in_ref);
  const Eigen::Vector3d roll_axis(cp, 0.0, -sp);
  const Eigen::Vector3d r_roll_joint =
    params.r_pitch_joint_in_ref + R_y * params.r_roll_in_pitch;
  const Eigen::Vector3d v_roll = roll_axis.cross(-r_roll_joint);

  Eigen::MatrixXd J_q = Eigen::MatrixXd::Zero(6, 2);
  J_q.block<3, 1>(0, 0) = v_pitch;
  J_q.block<3, 1>(3, 0) = omega_pitch;
  J_q.block<3, 1>(0, 1) = v_roll;
  J_q.block<3, 1>(3, 1) = roll_axis;

  const Eigen::MatrixXd J_temp = J_x * J_q;
  if (std::abs(J_theta.determinant()) < 1e-12) {
    return false;  // 曲柄-连杆奇异
  }
  // J[joint->motor]: dtheta/dq = J_theta^{-1} * (J_x * J_q)
  *J_out = J_theta.inverse() * J_temp;
  return true;
}

}  // namespace

ParallelKinematicsParams make_ankle_params(bool left_leg)
{
  ParallelKinematicsParams p;
  // 零位几何与 xjdl_description/*_ankle_parallel_kinematics_xjdl.xml 一致
  // (胫骨系)。0624 实机安装尺寸未标定, 为参考值。
  p.rod_len[0] = 0.313;
  p.rod_len[1] = 0.252;
  p.motor_abs_limit_rad = 1.7;
  p.joint_dir[0] = -1.0;
  p.joint_dir[1] = -1.0;
  // 与 MJCF ankle_pitch -> ankle_roll 的 pos="0 0 -0.016" 一致 (参考系原点在
  // roll 零位)
  p.r_pitch_joint_in_ref << 0.0, 0.0, 0.016;
  p.r_roll_in_pitch << 0.0, 0.0, -0.016;
  if (left_leg) {
    p.name = "left_ankle";
    p.r_A[0] << 0.067, 0.005, 0.323;
    p.r_B0[0] << 0.067, -0.019568, 0.32863;
    p.r_C_body[0] << 0.038, -0.0235, 0.015;
    p.r_A[1] << 0.059, -0.005, 0.263;
    p.r_B0[1] << 0.059, 0.019819, 0.267;
    p.r_C_body[1] << 0.038, 0.0235, 0.015;
  } else {
    p.name = "right_ankle";
    p.r_A[0] << 0.067, -0.005, 0.323;
    p.r_B0[0] << 0.067, 0.019568, 0.32863;
    p.r_C_body[0] << 0.038, 0.0235, 0.015;
    p.r_A[1] << 0.059, 0.005, 0.263;
    p.r_B0[1] << 0.059, -0.019819, 0.267;
    p.r_C_body[1] << 0.038, -0.0235, 0.015;
  }
  return p;
}

ParallelKinematicsParams make_waist_params()
{
  ParallelKinematicsParams p;
  p.name = "waist";
  // 与 waist_parallel_kinematics.xml 的 WaistA/B/C 一致 (两轴共点机构)。
  p.rod_len[0] = 0.073758;
  p.rod_len[1] = 0.073758;
  p.motor_abs_limit_rad = 1.5;
  p.joint_dir[0] = 1.0;
  p.joint_dir[1] = 1.0;
  p.r_A[0] << 0.03, 0.055, 0.044;
  p.r_B0[0] << 0.03, 0.0751, 0.0375;
  p.r_C_body[0] << 0.033, 0.07, -0.036;
  p.r_A[1] << 0.03, -0.055, 0.044;
  p.r_B0[1] << 0.03, -0.0751, 0.0375;
  p.r_C_body[1] << 0.033, -0.07, -0.036;
  return p;
}

bool jacobian(
  const ParallelKinematicsParams & params,
  double q_pitch, double q_roll,
  Eigen::Matrix2d * J)
{
  // IK 内部状态 (r_C/r_bar/r_rod) 供雅可比使用
  std::vector<Eigen::Vector3d> r_C_world, r_bar_world, r_rod_world;
  const double cp = std::cos(q_pitch);
  const double sp = std::sin(q_pitch);
  const double cr = std::cos(q_roll);
  const double sr = std::sin(q_roll);
  Eigen::Matrix3d R;
  R << cp, sp * sr, sp * cr,
       0.0, cr, -sr,
       -sp, cp * sr, cp * cr;

  for (int i = 0; i < 2; ++i) {
    // 求该链的曲柄角 (IK), 得到 r_B/r_rod
    double th = 0.0;
    if (!single_chain_theta(params, i, q_pitch, q_roll, params.joint_dir[i] * 0.0, &th)) {
      return false;
    }
    const Eigen::Vector3d r_A = params.r_A[i];
    const Eigen::Vector3d r_AB0 = params.r_B0[i] - r_A;
    const double cth = std::cos(th);
    const double sth = std::sin(th);
    Eigen::Matrix3d Rx;
    Rx << 1.0, 0.0, 0.0,
          0.0, cth, sth,
          0.0, -sth, cth;
    const Eigen::Vector3d r_B = r_A + Rx * r_AB0;
    const Eigen::Vector3d r_C_body_rot = R * params.r_C_body[i];
    const Eigen::Vector3d r_C =
      params.r_pitch_joint_in_ref + R * params.r_roll_in_pitch + r_C_body_rot;
    r_C_world.push_back(r_C);
    r_bar_world.push_back(r_B - r_A);
    r_rod_world.push_back(r_C - r_B);
  }
  return compute_jacobian(params, q_pitch, q_roll, r_C_world, r_bar_world, r_rod_world, J);
}

ParallelMechanism::ParallelMechanism(const ParallelKinematicsParams & params)
  : params_(params) {}

IkResult ParallelMechanism::ik(
  double q_pitch, double q_roll, const double theta_hint[2]) const
{
  IkResult result;
  for (int i = 0; i < 2; ++i) {
    const double hint = (theta_hint != nullptr) ? theta_hint[i] : 0.0;
    if (!single_chain_theta(params_, i, q_pitch, q_roll, hint, &result.theta[i])) {
      return result;  // ok=false
    }
  }
  result.ok = true;
  return result;
}

FkResult ParallelMechanism::fk(
  double theta_0, double theta_1,
  double hint_pitch, double hint_roll) const
{
  FkResult result;
  // 电机角越界直接失败
  if (std::abs(theta_0) > params_.motor_abs_limit_rad ||
      std::abs(theta_1) > params_.motor_abs_limit_rad) {
    result.failure_code = kFkMotorLimit;
    return result;
  }
  const Eigen::Vector2d theta_ref(theta_0, theta_1);
  Eigen::Vector2d x(hint_pitch, hint_roll);
  Eigen::Matrix2d J;
  for (int iter = 0; iter < kFkMaxIter; ++iter) {
    result.iterations = iter + 1;
    // 迭代当前 q 的 IK; 用目标电机角作为分支 hint, 保证 Newton 迭代中 IK
    // 始终与目标 θ 同一分支 (参考 CloseChainMapping 同款做法)
    IkResult ik_at_x = ik(x(0), x(1), theta_ref.data());
    if (!ik_at_x.ok) {
      result.failure_code = kFkBadGeometry;
      return result;
    }
    if (motor_limit_exceeded(Eigen::Vector2d(ik_at_x.theta[0], ik_at_x.theta[1]),
        params_.motor_abs_limit_rad)) {
      result.failure_code = kFkMotorLimit;
      return result;
    }
    Eigen::Vector2d f_error(theta_ref[0] - ik_at_x.theta[0],
                            theta_ref[1] - ik_at_x.theta[1]);
    if (f_error.cwiseAbs().maxCoeff() < kFkTolRad) {
      result.ok = true;
      result.q_pitch = x(0);
      result.q_roll = x(1);
      return result;
    }
    if (!jacobian(params_, x(0), x(1), &J) || J.hasNaN()) {
      result.failure_code = kFkJacobianNan;
      return result;
    }
    Eigen::Vector2d dq;
    if (!newton_step(J, f_error, &dq) || !std::isfinite(dq[0]) || !std::isfinite(dq[1])) {
      result.failure_code = kFkBadGeometry;
      return result;
    }
    dq *= std::min(1.0, kMaxNewtonStepRad / std::max(dq.norm(), 1e-12));
    x += dq;
  }
  result.failure_code = kFkNoConvergence;
  return result;
}

bool ParallelMechanism::joint_velocity(
  double q_pitch, double q_roll,
  double theta_dot_0, double theta_dot_1,
  double * q_pitch_dot, double * q_roll_dot) const
{
  Eigen::Matrix2d J;
  if (!jacobian(params_, q_pitch, q_roll, &J)) {
    return false;
  }
  if (std::abs(J.determinant()) < kDetJacobianEps) {
    return false;
  }
  const Eigen::Vector2d theta_dot(theta_dot_0, theta_dot_1);
  const Eigen::Vector2d q_dot = J.inverse() * theta_dot;
  *q_pitch_dot = q_dot(0);
  *q_roll_dot = q_dot(1);
  return true;
}

bool ParallelMechanism::joint_torque(
  double q_pitch, double q_roll,
  double tau_theta_0, double tau_theta_1,
  double * tau_q_pitch, double * tau_q_roll) const
{
  Eigen::Matrix2d J;
  if (!jacobian(params_, q_pitch, q_roll, &J)) {
    return false;
  }
  const Eigen::Vector2d tau_theta(tau_theta_0, tau_theta_1);
  const Eigen::Vector2d tau_q = J.transpose() * tau_theta;
  *tau_q_pitch = tau_q(0);
  *tau_q_roll = tau_q(1);
  return true;
}

}  // namespace ai_sapiens_sim2real::parallel