// Tenkun RTI：按 JSON/旧版拓扑配置实例化各 TenkunSlaveStation，并汇总全系统关节元数据。
#ifndef TENKUN_RTI_HPP_
#define TENKUN_RTI_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "transmit_soem_core.h"
#include "motor_control.h"
#include "tenkun_slave_station.hpp"

namespace XJDLRobot {

/** 单条关节在 RTI 中的静态描述（来自配置文件，含所属从站与网关逻辑通道）。 */
struct TenkunRtiJointInfo {
  int joint_index = 0;
  std::string name;
  int slave_index = 0;
  unsigned channel_1_to_12 = 0;
  uint16_t can_tx_id = 0;
};



class TenKunRti {
 public:
  /** EtherCAT 网卡：RTI JSON 的 interface 字段（根或 rti 包裹）；非 JSON 或空路径时用环境变量 TENKUN_EC_IF。 */
  explicit TenKunRti(const std::string& rti_config_path);
  ~TenKunRti();

  TenKunRti(const TenKunRti&) = delete;
  TenKunRti& operator=(const TenKunRti&) = delete;
  TenKunRti(TenKunRti&&) = delete;
  TenKunRti& operator=(TenKunRti&&) = delete;

  bool IsRunning() const { return getRunState(); }
  void StopRunning() { stopRun(); }
  bool IsStopped() const { return !getRunState(); }

  const std::string& EcInterfaceName() const { return ifname_; }

  const std::vector<TenkunRtiJointInfo>& Joints() const { return joints_; }

  const TenkunRtiJointInfo* FindJointByName(const std::string& name) const;

  /** 与配置中 joints_names 顺序、长度一致；按关节名将 cmd 下发到对应从站（混合阻抗）。size 不符返回 false。 */
  bool SetJointsCmdOrdered(const std::vector<MotorCmdType>& cmds);
  /** 与 joints_names 顺序、长度一致的反馈快照（无关节时为空）。 */
  std::vector<TenkunCanFdRxInfo> GetJointsStatusOrdered() const;
  const std::vector<std::string>& JointsCmdOrder() const { return joints_cmd_order_; }

  /** RTI JSON 可选字段 default_pos（弧度），与 joints_names 顺序、长度一致；未配置时为空。 */
  const std::vector<float>& DefaultPosRad() const { return default_pos_rad_; }

  void TenkunScanCanFdMotorIds(int slave_idx = 0);
  void TenkunBroadcastJointIdAllCanFdSlots(int slave_idx = 0);
  void TenkunEnqueueCih408SetJointId(int slave_idx, unsigned channel_1_to_15, uint16_t can_tx_id,
                                     uint8_t old_joint_id, uint8_t new_joint_id);
  /**
   * 按 joints_names 指令顺序下标（与 SetJointsCmdOrdered / GetJointsStatusOrdered 一致）对单关节下发 CiH408 零点标定 0x03。
   * 阻塞直至收到 0x83 应答且 Byte2=0x01（成功）；否则每 100ms 重发，最多 20 次后返回 false。
   */
  bool TenkunZeroCalibrateMotorOrdered(size_t motor_cmd_order_index);
  void TenkunLogMappedChannelHealth(int settle_ms = 800);

  TenkunSlaveStation* GetTenkunStation(int slave_index);
  const TenkunSlaveStation* GetTenkunStation(int slave_index) const;

  /** 对已配置从站索引 0..SLAVE_NUMBER-1 依次调用 RunControlCycle()（组令后每周期下发）。 */
  void RunControlAllSlaves();

  /** 各从站上已绑定电机全部 SetMotorEnabled(i, enabled)。 */
  void SetMotorEnabledAll(bool enabled = true);

  /** 当前 EtherCAT 会话对应的 RTI（构造完成至析构清理从站前有效）；供 SOEM 与 TenkunSlaveStation 内部路径使用。 */
  static TenKunRti* ActiveInstance();

  static void RegisterStation(int slave_idx, TenkunSlaveStation* p);
  static void UnregisterStation(int slave_idx, TenkunSlaveStation* p);
  static TenkunSlaveStation* FindStation(int slave_idx);
  static void DispatchProcessInputs(int slave_idx, const EtherCAT_Msg* rx);
  static uint8_t NextTxSeq(int slave_idx, unsigned channel_1_to_12);

 private:
  void PostStartHealthCheck();

  std::string ifname_;
  std::vector<std::unique_ptr<TenkunSlaveStation>> tenkun_stations_;
  std::vector<TenkunRtiJointInfo> joints_;
  /** RTI JSON 的 joints_names 顺序；旧版拓扑则为 joints_ 遍历顺序。 */
  std::vector<std::string> joints_cmd_order_;
  /** RTI JSON 可选 default_pos；非 JSON 或未写该字段时为空。 */
  std::vector<float> default_pos_rad_;

  std::array<TenkunSlaveStation*, SLAVE_NUMBER> station_slots_{};
  std::array<std::array<uint8_t, TENKUN_MOTORS_PER_SLAVE>, SLAVE_NUMBER> fallback_seq_{};

  static std::atomic<TenKunRti*> s_active_;
};

}  // namespace XJDLRobot

#endif  // TENKUN_RTI_HPP_
