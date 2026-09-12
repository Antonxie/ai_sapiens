// Copyright 2026 ai_sapiens_real contributors
// RealRobotSystem 实现 (骨架): 配置解析 / TenKunRti 全身调度 / 并联 IK-FK /
// 真机 PD 第 2 段 (29Dof RunLegMotor 逻辑) / ros2_control read-write。
// 说明: 主循环由 ros2_control 的 update loop 驱动 (read/write 周期性回调),
//       rti 各从站周期刷新由 SetJointsCmdOrdered/RunControlAllSlaves 完成。

#include "real_robot_hardware_interface/real_robot_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include <pluginlib/class_list_macros.hpp>

#include "tenkun_rti.hpp"
#include "motor_control.h"        // MotorCmdType / TenkunCanFdRxInfo (tenkun-soem)

namespace real_robot_hardware_interface
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using hardware_interface::StateInterface;
using hardware_interface::CommandInterface;

constexpr double kDefaultCommandRate = 500.0;

// ---------------- lifecycle ----------------

CallbackReturn RealRobotSystem::on_init(const hardware_interface::HardwareInfo & hw_info)
{
  try {
    if (hardware_interface::SystemInterface::on_init(hw_info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("RealRobotSystem"), "on_init: %s", e.what());
    return CallbackReturn::ERROR;
  }

  logger_ = rclcpp::get_logger("RealRobotSystem");
  joint_names_.resize(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    joint_names_[i] = info_.joints[i].name;
  }
  dof_to_motor_slot_.resize(joint_names_.size());
  for (size_t i = 0; i < dof_to_motor_slot_.size(); ++i) {
    dof_to_motor_slot_[i] = static_cast<int>(i);  // 默认槽位 = 全局 DOF 索引
  }

  // schema: URDF hardware param "schema_path"
  for (const auto & p : info_.hardware_parameters) {
    if (p.first == "schema_path") {
      schema_path_ = p.second;
    }
  }
  if (!schema_path_.empty() && !load_schema(schema_path_)) {
    RCLCPP_FATAL(logger_, "failed to load schema %s", schema_path_.c_str());
    return CallbackReturn::ERROR;
  }
  // 反馈数组先分配, export 时需要稳定地址
  fbk_pos_.assign(joint_names_.size(), 0.0);
  fbk_vel_.assign(joint_names_.size(), 0.0);
  fbk_tau_.assign(joint_names_.size(), 0.0);
  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> RealRobotSystem::export_state_interfaces()
{
  std::vector<StateInterface> ret;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & n = info_.joints[i].name;
    for (const auto & si : info_.joints[i].state_interfaces) {
      double * p = nullptr;
      if (si.name == "position") { p = &fbk_pos_[i]; }
      else if (si.name == "velocity") { p = &fbk_vel_[i]; }
      else if (si.name == "effort") { p = &fbk_tau_[i]; }
      ret.emplace_back(n, si.name, p);
    }
  }
  return ret;
}

std::vector<CommandInterface> RealRobotSystem::export_command_interfaces()
{
  std::vector<CommandInterface> ret;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & n = info_.joints[i].name;
    for (const auto & ci : info_.joints[i].command_interfaces) {
      double * p = nullptr;
      if (ci.name == "position") { p = &cmd_pos_[i]; cmd_pos_h_.push_back(p); }
      else if (ci.name == "feedforward") { p = &cmd_ff_[i]; cmd_ff_h_.push_back(p); }
      else if (ci.name == "proportional") { p = &cmd_kp_[i]; cmd_kp_h_.push_back(p); }
      else if (ci.name == "derivative") { p = &cmd_kd_[i]; cmd_kd_h_.push_back(p); }
      ret.emplace_back(n, ci.name, p);
    }
  }
  return ret;
}

CallbackReturn RealRobotSystem::on_configure(const rclcpp_lifecycle::State & /*prev*/)
{
  const size_t n = joint_names_.size();
  cmd_pos_.assign(n, 0.0); cmd_vel_.assign(n, 0.0); cmd_ff_.assign(n, 0.0);
  cmd_kp_.assign(n, 0.0); cmd_kd_.assign(n, 0.0);
  fbk_pos_.assign(n, 0.0); fbk_vel_.assign(n, 0.0); fbk_tau_.assign(n, 0.0);
  return CallbackReturn::SUCCESS;
}

CallbackReturn RealRobotSystem::on_activate(const rclcpp_lifecycle::State & /*prev*/)
{
  /// 构造 EtherCAT + 全身调度 (enp4s0, 3 从站, rti JSON)
  // rti_config_path 从 schema ethercat.rti_config_path 读 (load_schema 存到成员? 简化: 由 schema_path 所在目录推断)
  // 骨架: 真实部署时以 schema.ethercat.rti_config_path 为准; 这里用默认相对路径.
  try {
    // rti_ = std::make_unique<XJDLRobot::TenKunRti>(rti_json_path_);
    // rti_->SetMotorEnabledAll(true);
    RCLCPP_WARN(logger_, "RealRobotSystem: TenKunRti activation placeholder (need rti json path)");
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger_, "activate failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  running_.store(true);
  last_write_ = std::chrono::steady_clock::now();
  return CallbackReturn::SUCCESS;
}

CallbackReturn RealRobotSystem::on_deactivate(const rclcpp_lifecycle::State & /*prev*/)
{
  running_.store(false);
  if (rti_) {
    rti_->SetMotorEnabledAll(false);
  }
  return CallbackReturn::SUCCESS;
}

// ---------------- read / write ----------------

return_type RealRobotSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!running_.load() || !rti_) {
    return return_type::OK;
  }
  const auto st = rti_->GetJointsStatusOrdered();
  for (size_t i = 0; i < st.size() && i < fbk_pos_.size(); ++i) {
    fbk_pos_[i] = st[i].pos_rad;    // 字段名以 tenkun_cih408/TenkunCanFdRxInfo 为准
    fbk_vel_[i] = st[i].vel_rad_s;
    fbk_tau_[i] = st[i].cur_a * kt_[std::min(i, kt_.size() - 1)];
  }
  motor_to_dof_fk();                          // 并联 DOF 还原
  for (size_t i = 0; i < st_pos_h_.size() && i < st_vel_h_.size(); ++i) {
    if (st_pos_h_[i]) { *st_pos_h_[i] = fbk_pos_[i]; }
    if (st_vel_h_[i]) { *st_vel_h_[i] = fbk_vel_[i]; }
    if (i < st_eff_h_.size() && st_eff_h_[i]) { *st_eff_h_[i] = fbk_tau_[i]; }
  }
  return return_type::OK;
}

return_type RealRobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!running_.load() || !rti_) {
    return return_type::OK;
  }
  const auto now = std::chrono::steady_clock::now();
  if (command_rate_s_ > 0.0 &&
      std::chrono::duration<double>(now - last_write_).count() < 1.0 / command_rate_s_) {
    return return_type::OK;                    // 500Hz 节流
  }
  last_write_ = now;

  // 命令接口 -> MotorCommand 字段
  const size_t n = cmd_pos_.size();
  if (cmd_pos_h_.size() >= n && cmd_kp_h_.size() >= n) {
    for (size_t i = 0; i < n; ++i) {
      if (cmd_pos_h_[i]) { cmd_pos_[i] = *cmd_pos_h_[i]; }
      if (cmd_ff_h_[i])  { cmd_ff_[i]  = *cmd_ff_h_[i]; }
      if (cmd_kp_h_[i])  { cmd_kp_[i]  = *cmd_kp_h_[i]; }
      if (cmd_kd_h_[i])  { cmd_kd_[i]  = *cmd_kd_h_[i]; }
    }
  }

  dof_to_motor_ik();                           // 并联 DOF 目标 -> 电机角
  apply_host_pd();                             // 第 2 段 + kp/kd*10

  // 组 29 个 MotorCmdType 按 rti 顺序下发
  std::vector<MotorCmdType> cmds(n);
  for (size_t i = 0; i < n; ++i) {
    cmds[i].kp  = static_cast<float>(cmd_kp_[i]);
    cmds[i].kd  = static_cast<float>(cmd_kd_[i]);
    cmds[i].pos = static_cast<float>(cmd_pos_[i]);
    cmds[i].spd = static_cast<float>(cmd_vel_[i]);
    cmds[i].tor = static_cast<float>(cmd_ff_[i]);
  }
  rti_->SetJointsCmdOrdered(cmds);
  rti_->RunControlAllSlaves();
  return return_type::OK;
}

// ---------------- 并联解算 ----------------

void RealRobotSystem::dof_to_motor_ik()
{
  if (!parallel_enabled_) {
    return;
  }
  for (const auto & g : parallel_groups_) {
    auto * mech = (g.pitch_slot < 6) ? mech_left_ankle_.get() :
                  (g.pitch_slot < 12) ? mech_right_ankle_.get() : mech_waist_.get();
    if (!mech) {
      continue;
    }
    double th[2] = {0.0, 0.0};
    const double qp = cmd_pos_[g.pitch_dof];
    const double qr = cmd_pos_[g.roll_dof];
    // theta_hint 用上帧 FK 解
    const auto ik = mech->ik(qp, qr, nullptr);
    if (ik.ok) {
      cmd_pos_[g.pitch_slot] = ik.theta[0];
      cmd_pos_[g.roll_slot] = ik.theta[1];
    }
  }
}

void RealRobotSystem::motor_to_dof_fk()
{
  if (!parallel_enabled_) {
    return;
  }
  for (const auto & g : parallel_groups_) {
    auto * mech = (g.pitch_slot < 6) ? mech_left_ankle_.get() :
                  (g.pitch_slot < 12) ? mech_right_ankle_.get() : mech_waist_.get();
    if (!mech) {
      continue;
    }
    const auto fk = mech->fk(fbk_pos_[g.pitch_slot], fbk_pos_[g.roll_slot],
                             fbk_pos_[g.pitch_dof], fbk_pos_[g.roll_dof]);
    if (fk.ok) {
      fbk_pos_[g.pitch_dof] = fk.q_pitch;
      fbk_pos_[g.roll_dof] = fk.q_roll;
    }
  }
}

// ---------------- 真机 PD 第 2 段 (29Dof RunLegMotor) ----------------

void RealRobotSystem::apply_host_pd()
{
  const double s_kp = host_pd_groups_.empty() ? 10.0 : 10.0;   // 全员 ×10 (schema 可改 scale)
  const double s_kd = host_pd_groups_.empty() ? 10.0 : 10.0;
  for (const auto & g : host_pd_groups_) {
    if (!g.enabled) {
      continue;
    }
    for (size_t i = 0; i < g.dofs.size(); ++i) {
      const int dof = g.dofs[i];
      const double tau = g.kp_scale * (cmd_pos_[dof] - fbk_pos_[dof]) +
                         g.kd_scale * (0.0 - fbk_vel_[dof]);
      const double clamped = std::clamp(tau, -g.torque_clamp_nm, g.torque_clamp_nm);
      cmd_ff_[dof] += clamped;
      cmd_kp_[dof] = g.cmd_kp;
      cmd_kd_[dof] = g.cmd_kd;
    }
  }
  // 全员 kp/kd * 10 (CiH408 固件阻抗单位配合, 勿改)
  for (size_t i = 0; i < cmd_kp_.size(); ++i) {
    cmd_kp_[i] *= s_kp;
    cmd_kd_[i] *= s_kd;
  }
}

// ---------------- schema 解析 ----------------

bool RealRobotSystem::load_schema(const std::string & path)
{
  try {
    const auto node = YAML::LoadFile(path);
    // ethercat
    auto ec = node["ethercat"];
    if (ec) {
      if (ec["control_frequency_hz"]) {
        command_rate_s_ = ec["control_frequency_hz"].as<double>();
      }
    }
    if (command_rate_s_ <= 0.0) {
      command_rate_s_ = kDefaultCommandRate;
    }
    load_parallel(node["parallel"]);
    load_kt(node["kt_nm_per_a"]);
    load_tau_limit(node["tau_limit_nm"]);
    load_host_pd(node["host_pd"]);
    load_stand_gains(node["stand_gains"]);
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "load_schema %s: %s", path.c_str(), e.what());
    return false;
  }
}

void RealRobotSystem::load_parallel(const YAML::Node & node)
{
  if (!node || !node.IsMap()) {
    return;
  }
  parallel_enabled_ = node["enabled"] && node["enabled"].as<bool>();
  if (parallel_enabled_) {
    // 用代码内置 XJDL 参考几何 (geometry 字段后续从 schema 读取)
    mech_left_ankle_ = std::make_unique<ai_sapiens_sim2real::parallel::ParallelMechanism>(
      ai_sapiens_sim2real::parallel::make_ankle_params(true));
    mech_right_ankle_ = std::make_unique<ai_sapiens_sim2real::parallel::ParallelMechanism>(
      ai_sapiens_sim2real::parallel::make_ankle_params(false));
    mech_waist_ = std::make_unique<ai_sapiens_sim2real::parallel::ParallelMechanism>(
      ai_sapiens_sim2real::parallel::make_waist_params());

    auto add_group = [this](const YAML::Node & g) {
      if (!g) {
        return;
      }
      ParallelGroup pg;
      pg.pitch_dof = g["pitch_dof"].as<int>();
      pg.roll_dof = g["roll_dof"].as<int>();
      pg.pitch_slot = dof_to_motor_slot_.empty() ? pg.pitch_dof : dof_to_motor_slot_[pg.pitch_dof];
      pg.roll_slot = dof_to_motor_slot_.empty() ? pg.roll_dof : dof_to_motor_slot_[pg.roll_dof];
      parallel_groups_.push_back(pg);
    };
    add_group(node["left_ankle"]);
    add_group(node["right_ankle"]);
    add_group(node["waist"]);
  }
}

void RealRobotSystem::load_kt(const YAML::Node & node)
{
  const size_t n = joint_names_.size();
  kt_.assign(n, node["default"] ? node["default"].as<double>() : 2.7);
  if (node["per_joint"]) {
    const auto & v = node["per_joint"];
    for (size_t i = 0; i < v.size() && i < n; ++i) {
      kt_[i] = v[i].as<double>();
    }
  }
}

void RealRobotSystem::load_tau_limit(const YAML::Node & node)
{
  const size_t n = joint_names_.size();
  tau_limit_nm_.assign(n, 200.0);
  if (node && node.IsSequence() && node[0] && node[0].size() == n) {
    for (size_t i = 0; i < n; ++i) {
      tau_limit_nm_[i] = node[0][i].as<double>();
    }
  }
}

void RealRobotSystem::load_host_pd(const YAML::Node & node)
{
  host_pd_groups_.clear();
  if (!node || !node.IsMap()) {
    return;
  }
  auto load_group = [this](const YAML::Node & g) {
    HostPdGroup grp;
    grp.enabled = g["enabled"] ? g["enabled"].as<bool>() : true;
    if (g["dofs"]) {
      for (const auto & d : g["dofs"]) {
        grp.dofs.push_back(d.as<int>());
      }
    }
    if (g["kp_scale"]) { grp.kp_scale = g["kp_scale"].as<double>(); }
    if (g["kd_scale"]) { grp.kd_scale = g["kd_scale"].as<double>(); }
    if (g["torque_clamp_nm"]) { grp.torque_clamp_nm = g["torque_clamp_nm"].as<double>(); }
    if (g["cmd_kp"]) { grp.cmd_kp = g["cmd_kp"].as<double>(); }
    if (g["cmd_kd"]) { grp.cmd_kd = g["cmd_kd"].as<double>(); }
    host_pd_groups_.push_back(grp);
  };
  if (node["ankle"]) { load_group(node["ankle"]); }
  if (node["waist"]) { load_group(node["waist"]); }
}

void RealRobotSystem::load_stand_gains(const YAML::Node & node)
{
  const size_t n = joint_names_.size();
  stand_kp_.assign(n, 40.0);
  stand_kd_.assign(n, 1.0);
  if (node && node["kp"]) {
    for (size_t i = 0; i < node["kp"].size() && i < n; ++i) {
      stand_kp_[i] = node["kp"][i].as<double>();
    }
  }
  if (node && node["kd"]) {
    for (size_t i = 0; i < node["kd"].size() && i < n; ++i) {
      stand_kd_[i] = node["kd"][i].as<double>();
    }
  }
}

}  // namespace real_robot_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  real_robot_hardware_interface::RealRobotSystem,
  hardware_interface::SystemInterface)