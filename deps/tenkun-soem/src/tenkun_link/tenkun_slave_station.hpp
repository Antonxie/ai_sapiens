// 以单个 EtherCAT 从站为单位的 Tenkun 网关 + CiH408：配置解析、PDO 语义、反馈缓存。
// 下发经 SoemSendToQueue（每从站一条 tx 队列，transmit_soem_core.cpp）；收由 SOEM 周期线程读 inputs 后调用 FeedProcessInputs。
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#ifdef __cplusplus
extern "C" {
#endif
#include "motor_control.h"
#ifdef __cplusplus
}
#endif

#include "tenkun_gateway_pdo.h"
#include "tenkun_motor_cmd.h"

namespace XJDLRobot {

struct TenkunMotorBinding {
  unsigned channel_1_to_12 = 0;
  uint16_t can_tx_id = 0;
};

struct TenkunSlaveStationConfig {
  int slave_index = 0;
  std::vector<TenkunMotorBinding> motors;
};

class TenkunSlaveStation {
 public:
  explicit TenkunSlaveStation(TenkunSlaveStationConfig config);
  ~TenkunSlaveStation();

  TenkunSlaveStation(const TenkunSlaveStation&) = delete;
  TenkunSlaveStation& operator=(const TenkunSlaveStation&) = delete;

  struct TopologyParseOutcome {
    bool ok = false;
    std::vector<TenkunSlaveStationConfig> station_configs;
  };

  static TopologyParseOutcome ParseTopologyFile(const std::string& path);

  int SlaveIndex() const { return slave_index_; }
  size_t MotorCount() const { return motors_.size(); }
  const std::vector<TenkunMotorBinding>& Bindings() const { return motors_; }

  void QueryMotorStatus(size_t motor_index);
  TenkunCanFdRxInfo GetMotorStatus(size_t motor_index) const;
  /** 清除该电机槽位缓存中的零点标定应答成功位，避免沿用上一次标定的残留状态。 */
  void ClearZeroCalibrateAckState(size_t motor_index);
  TenkunCanFdRxInfo LastFeedbackForChannel(unsigned channel_1_to_12) const;
  uint16_t ResolveCanIdForChannel(unsigned channel_1_to_12) const;

  /** 据 CiH408 ENABLE 应答中的 enable_on；周期性 STATUS_16 会保留该字段直至下一次 ENABLE 应答。未收到过 ENABLE 应答前为 false。 */
  bool IsMotorFeedbackEnabled(size_t motor_index) const;

  void SetMotorEnabled(size_t motor_index, bool enabled);
  void SetHybridImpedance(size_t motor_index, float kp, float kd, float pos_rad, float vel_rad_s,
                          float torque_ff_nm);

  void RunControlCycle();

  /** 本从站：推入 Tx 队列（内部 SoemSendToQueue(slave_index_, msg)）。 */
  void EnqueueTx(const std::shared_ptr<EtherCAT_Msg>& msg);

  /**
   * 在本从站上下文中组包到 msg；cmd.slave 应与 SlaveIndex() 一致，否则打印警告并以本从站为准。
   * submit_to_queue 为 true 时调用 EnqueueTx(msg)。
   */
  void PackMotorCmdInto(const std::shared_ptr<EtherCAT_Msg>& msg, MotorCmdType cmd, bool submit_to_queue);

  /** 本从站已绑定通道整帧 0x17 查询入队（不检查 holdoff）。 */
  void EnqueueAllMappedChannelQueries();

  /** SOEM 收到 inputs 后调用（经 TenKunRti::DispatchProcessInputs）。 */
  void FeedProcessInputs(const EtherCAT_Msg& rx);

  uint8_t BumpSeq(unsigned channel_1_to_12);

  /** 整帧清零后对本从站已绑定通道写入 0x17 查询。 */
  void FillAllQuerySlots(EtherCAT_Msg* tx) const;

  void ResetJointIdScan();
  void PrintJointIdScanResults() const;

 private:
  void RebuildCommandArrays();
  void UnpackOneSlot(unsigned channel_1_to_12, const TenkunGatewaySlotView* sv);

  int slave_index_;
  std::vector<TenkunMotorBinding> motors_;

  std::array<TenkunCanFdRxInfo, TENKUN_MOTORS_PER_SLAVE> slot_rx_{};
  mutable std::mutex rx_mu_;

  std::array<uint8_t, TENKUN_MOTORS_PER_SLAVE> seq_tx_{};
  std::array<uint8_t, TENKUN_MOTORS_PER_SLAVE> query_armed_{};
  std::array<uint8_t, TENKUN_MOTORS_PER_SLAVE> query_complete_{};
  std::array<uint8_t, TENKUN_MOTORS_PER_SLAVE> id_scan_hit_{};
  std::array<uint8_t, TENKUN_MOTORS_PER_SLAVE> id_scan_joint_id_{};

  std::vector<uint8_t> enabled_desired_;
  std::vector<uint8_t> enable_pending_;
  std::vector<float> hybrid_kp_;
  std::vector<float> hybrid_kd_;
  std::vector<float> hybrid_pos_;
  std::vector<float> hybrid_vel_;
  std::vector<float> hybrid_torque_;
  mutable std::mutex mu_;
};

}  // namespace XJDLRobot
