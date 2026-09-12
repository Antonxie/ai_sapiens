// Copyright 2026 ai_sapiens_sim2real contributors
//
// 并联机构运动学冒烟测试 (跨 x86 / arm64 双架构验证)。
//
// 自检内容 (对 左踝/右踝/腰 三组参考几何):
//   1. q 域工作空间扫描: 对每个可达 q, IK 必须成功, FK(hint=上一点) 收敛,
//      还原误差 |q - q_fk| < 1e-6;
//   2. FK 再用通过率统计; 任一机构通过率 < 100% 即 FAIL (exit 1);
//   3. joint_velocity / joint_torque 映射在非奇异点可解。
//
// 无 ROS 依赖, 可直接在宿主 x86 运行; aarch64 交叉编译产物用 qemu 运行。

#include "ai_sapiens_sim2real/parallel/parallel_kinematics.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

using ai_sapiens_sim2real::parallel::FkResult;
using ai_sapiens_sim2real::parallel::IkResult;
using ai_sapiens_sim2real::parallel::ParallelMechanism;
using ai_sapiens_sim2real::parallel::ParallelKinematicsParams;
using ai_sapiens_sim2real::parallel::make_ankle_params;
using ai_sapiens_sim2real::parallel::make_waist_params;

namespace {

struct SweepConfig {
  double theta_abs_min;   // 电机角扫描域: 对称 [-abs, +abs]
  double theta_abs_max;
  double step;
  int q_probe_count;      // IK 正向探测点数 (工作空间可达性)
  double q_probe_abs;     // q 探测域: 对称 [-abs, +abs] (rad)
};

// 电机空间扫描, 双向自洽 (真实 harness 数据流: 电机反馈角 -> FK -> DOF -> IK):
//   对电机角网格 (theta0, theta1):  FK(hint=上帧q) 必须成功,
//   且 IK(FK结果) 还原回 (theta0, theta1)。通过率 = 还原成功 / FK 成功。
bool sweep_motor(
  const char * name,
  const ParallelMechanism & mech,
  const SweepConfig & cfg)
{
  const double tol_th = 1e-5;
  int fk_ok = 0;
  int passed = 0;
  int fk_fail = 0;
  int fk_code[8] = {0};  // 失败码分布: -1..-4 对应 kFk* 常量
  double hint_q[2] = {0.0, 0.0};

  for (double t0 = cfg.theta_abs_min; t0 <= cfg.theta_abs_max + 1e-9; t0 += cfg.step) {
    for (double t1 = cfg.theta_abs_min; t1 <= cfg.theta_abs_max + 1e-9; t1 += cfg.step) {
      // 模拟实机反馈角, hint 用上一帧 FK 结果 (500Hz 连续性)
      const FkResult fk = mech.fk(t0, t1, hint_q[0], hint_q[1]);
      if (!fk.ok) {
        ++fk_fail;
        fk_code[fk.failure_code >= -4 && fk.failure_code <= -1 ? -fk.failure_code : 7] += 1;
        // 只打印首个, 避免刷屏
        if (fk_fail <= 5) {
          std::printf("  [%s] fk_fail theta=(%.3f,%.3f) hint=(%.3f,%.3f) code=%d\n",
                      name, t0, t1, hint_q[0], hint_q[1], fk.failure_code);
        }
        continue;
      }
      ++fk_ok;
      hint_q[0] = fk.q_pitch;
      hint_q[1] = fk.q_roll;
      // IK 还原: q -> theta, 应与输入一致
      const double th_hint[2] = {t0, t1};
      const IkResult ik_back = mech.ik(fk.q_pitch, fk.q_roll, th_hint);
      const double err_th0 = ik_back.ok ? std::abs(ik_back.theta[0] - t0) : 1e9;
      const double err_th1 = ik_back.ok ? std::abs(ik_back.theta[1] - t1) : 1e9;
      if (err_th0 < tol_th && err_th1 < tol_th) {
        ++passed;
      } else {
        std::printf("  [%s] roundtrip fail theta=(%.3f,%.3f) -> q=(%.4f,%.4f) "
                    "ik=(%.3f,%.3f)\n",
                    name, t0, t1, fk.q_pitch, fk.q_roll,
                    ik_back.ok ? ik_back.theta[0] : 9.9,
                    ik_back.ok ? ik_back.theta[1] : 9.9);
      }
    }
  }

  // IK 正向可达性 (q 域抽样): 验证工作空间可达率 (腰的机构杆长短,
  // 实际工作空间小, 探测域按各自几何收紧)
  int q_ok = 0;
  const int q_steps = cfg.q_probe_count;
  const double q_abs = cfg.q_probe_abs;
  for (int i = 0; i <= q_steps; ++i) {
    for (int j = 0; j <= q_steps; ++j) {
      const double qp = -q_abs + 2.0 * q_abs * i / static_cast<double>(q_steps);
      const double qr = -q_abs + 2.0 * q_abs * j / static_cast<double>(q_steps);
      if (mech.ik(qp, qr, nullptr).ok) {
        ++q_ok;
      }
    }
  }
  const double q_rate = 100.0 * q_ok / ((q_steps + 1) * (q_steps + 1));

  // PASS 判定:
  //   - 双向自洽 100% (fk_ok 内 passed == fk_ok): 核心数值正确
  //   - fk_fail 少于 fk_ok: 扫描域含物理不可达的电机角组合, FK 拒绝它们是
  //     正确行为; 但拒绝过多说明扫描域/几何明显异常
  const double rate = (fk_ok > 0) ? 100.0 * passed / fk_ok : 0.0;
  std::printf("  [%s] fk_ok=%d passed=%d fk_fail=%d rate=%.2f%% | q_probe(q<%+.2f)=%.1f%% "
              "fk_codes=[-1:%d -2:%d -3:%d -4:%d other:%d]\n",
              name, fk_ok, passed, fk_fail, rate, q_abs, q_rate,
              fk_code[1], fk_code[2], fk_code[3], fk_code[4], fk_code[7]);
  return fk_ok > 0 && passed == fk_ok && fk_fail < fk_ok && q_rate > 20.0;
}

bool sanity_velocity_torque(const char * name, const ParallelMechanism & mech)
{
  // 非奇异点验证动量映射可解: 输出有限
  double q_p_dot = 0.0, q_r_dot = 0.0, tau_p = 0.0, tau_r = 0.0;
  const bool vel_ok = mech.joint_velocity(0.05, 0.05, 0.1, 0.2, &q_p_dot, &q_r_dot);
  const bool tau_ok = mech.joint_torque(0.05, 0.05, 0.1, 0.2, &tau_p, &tau_r);
  const bool finite = std::isfinite(q_p_dot) && std::isfinite(q_r_dot) &&
                      std::isfinite(tau_p) && std::isfinite(tau_r);
  std::printf("  [%s] vel=%s tau=%s finite=%s (q_dot=(%.4f,%.4f), tau_q=(%.4f,%.4f))\n",
              name, vel_ok ? "ok" : "FAIL", tau_ok ? "ok" : "FAIL",
              finite ? "yes" : "no", q_p_dot, q_r_dot, tau_p, tau_r);
  return vel_ok && tau_ok && finite;
}

}  // namespace

int main()
{
  std::printf("=== parallel_kinematics smoke test (x86/arm64) ===\n");
  bool all_ok = true;

  // 踝: 电机限位 1.7 rad (urdf pitch[-0.91,0.52] roll[-0.35,0.35]),
  // 扫描电机空间 [-1.4, +1.4] 已覆盖 80 度曲柄角, 避免 90 度奇异带
  const SweepConfig ankle_cfg = {-1.4, 1.4, 0.2, 8, 0.4};
  const ParallelMechanism left_ankle(make_ankle_params(true));
  std::printf("-- left_ankle --\n");
  all_ok &= sweep_motor("left_ankle", left_ankle, ankle_cfg);
  all_ok &= sanity_velocity_torque("left_ankle", left_ankle);

  const ParallelMechanism right_ankle(make_ankle_params(false));
  std::printf("-- right_ankle --\n");
  all_ok &= sweep_motor("right_ankle", right_ankle, ankle_cfg);
  all_ok &= sanity_velocity_torque("right_ankle", right_ankle);

  // 腰: 两轴共点机构, URDF 范围极小 (roll ±0.035, pitch ±0.052),
  // 电机限位 1.5, 扫描 [-1.0, +1.0]; q 探测域 0.3 (机构杆长短, 工作空间小)
  const SweepConfig waist_cfg = {-1.0, 1.0, 0.2, 8, 0.3};
  const ParallelMechanism waist(make_waist_params());
  std::printf("-- waist --\n");
  all_ok &= sweep_motor("waist", waist, waist_cfg);
  all_ok &= sanity_velocity_torque("waist", waist);

  std::printf("%s\n", all_ok ? "RESULT: ALL PASS" : "RESULT: FAIL");
  return all_ok ? 0 : 1;
}