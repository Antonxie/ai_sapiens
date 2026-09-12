// Copyright 2026 ai_sapiens_real contributors
//
// RealRobotSystem: 0624 真机 hardware_interface 插件 (ros2_control SystemInterface)。
// 替换 sim 的 MujocoSystem, 把策略的 29 维 DOF 阻抗命令落到真机电机:
//   write(): DOF 命令 -> (踝/腰并联 IK) -> MotorCommand -> 真机 PD 第 2 段(§8.5) ->
//            kp/kd*10 -> TenKunRti::SetJointsCmdOrdered -> SOEM 3 从站
//   read() : TenKunRti::GetJointsStatusOrdered -> (并联 FK) -> DOF 角回填 /joint_states
//
// 构建依赖(独立于 sim 项目, 避免环境串扰):
//   deps/tenkun-soem  (SOEM + tenkun_cih408 + tenkun_rti 全身调度)
//   并行解算: ai_sapiens_sim2real::parallel::ParallelMechanism (同 workspace 源码)
//   配置:     config/real_robot_0624.yaml (PD schema / Kt / 路由 / IMU)

#ifndef REAL_ROBOT_HARDWARE_INTERFACE__REAL_ROBOT_SYSTEM_HPP_
#define REAL_ROBOT_HARDWARE_INTERFACE__REAL_ROBOT_SYSTEM_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp/logger.hpp>
#include <yaml-cpp/yaml.h>

#include "ai_sapiens_sim2real/parallel/parallel_kinematics.hpp"

namespace XJDLRobot
{
class TenKunRti;
}  // namespace XJDLRobot

namespace real_robot_hardware_interface
{

class RealRobotSystem : public hardware_interface::SystemInterface
{
public:
  RealRobotSystem() = default;   // pluginlib 需要默认构造
  ~RealRobotSystem() override = default;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hw_info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct HostPdGroup
  {
    bool enabled{false};
    std::vector<int> dofs;          // 附加 PD 的全局 DOF 索引
    double kp_scale{1.0}, kd_scale{1.0};
    double torque_clamp_nm{50.0};
    double cmd_kp{0.0}, cmd_kd{0.02};
  };

  // schema 解析
  bool load_schema(const std::string & path);
  void load_joint_mapping(const YAML::Node & node);      // URDF joint 索引 -> schema
  void load_parallel(const YAML::Node & node);
  void load_kt(const YAML::Node & node);
  void load_tau_limit(const YAML::Node & node);
  void load_host_pd(const YAML::Node & node);
  void load_stand_gains(const YAML::Node & node);

  // 命令/反馈转换
  void dof_to_motor_ik();                       // write 前: 并联 DOF 目标 -> 电机角
  void motor_to_dof_fk();                       // read 后: 电机反馈 -> 并联 DOF
  void apply_host_pd();                         // 第 2 段附加 PD + kp/kd*10

  rclcpp::Logger logger_{rclcpp::get_logger("RealRobotSystem")};
  std::string schema_path_{};

  // 关节/槽位
  std::vector<std::string> joint_names_;        // URDF 29 序 (hw_info)
  std::vector<int> dof_to_motor_slot_;          // 全局 DOF -> rti joints_cmd_order_ 下标 (默认 idx=i)

  // 电机中间状态 (MotorCmdType 语义)
  std::vector<double> cmd_pos_, cmd_vel_, cmd_ff_, cmd_kp_, cmd_kd_;
  std::vector<double> fbk_pos_, fbk_vel_, fbk_tau_;

  // 并联解算 (腰/左右踝)
  bool parallel_enabled_{false};
  std::unique_ptr<ai_sapiens_sim2real::parallel::ParallelMechanism> mech_left_ankle_;
  std::unique_ptr<ai_sapiens_sim2real::parallel::ParallelMechanism> mech_right_ankle_;
  std::unique_ptr<ai_sapiens_sim2real::parallel::ParallelMechanism> mech_waist_;
  // 全局 DOF -> 组内坐标: {pitch_dof, roll_dof, pitch 指向的电机槽, roll 指向的电机槽}
  struct ParallelGroup { int pitch_dof, roll_dof, pitch_slot, roll_slot; };
  std::vector<ParallelGroup> parallel_groups_;

  // 真机 PD 第 2 段
  std::vector<HostPdGroup> host_pd_groups_;

  // Kt / 力矩限
  std::vector<double> kt_;
  std::vector<double> tau_limit_nm_;

  // 站姿增益
  std::vector<double> stand_kp_, stand_kd_;

  // ROS2_control command/state interface 句柄
  std::vector<double *> cmd_pos_h_, cmd_ff_h_, cmd_kp_h_, cmd_kd_h_;
  std::vector<double *> st_pos_h_, st_vel_h_, st_eff_h_;

  std::unique_ptr<XJDLRobot::TenKunRti> rti_;
  std::atomic<bool> running_{false};            // on_activate 置真, 触发 read/write 生效
  double command_rate_s_{0.0};
  std::chrono::steady_clock::time_point last_write_{};
};

}  // namespace real_robot_hardware_interface

#endif  // REAL_ROBOT_HARDWARE_INTERFACE__REAL_ROBOT_SYSTEM_HPP_