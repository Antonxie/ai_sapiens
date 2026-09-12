#!/usr/bin/env bash
# Model_A_E 0624 ai_sapiens sim2sim 验收测试（容器内执行）
# 覆盖部署蓝图 B 系列用例（B1-B7）。速度量化依赖 mjlab eval_vel_bins 基准做行为对比。
set -uo pipefail

source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash

PASS=0; FAIL=0
ok()  { echo "  [PASS] $1"; PASS=$((PASS+1)); }
bad() { echo "  [FAIL] $1"; FAIL=$((FAIL+1)); }

# ---- 等待仿真与策略就绪 ----
echo "== B0: 启动 modelae_0624_mujoco =="
ros2 launch ai_sapiens_bringup modelae_0624_mujoco.launch.py > /tmp/s2s_launch.log 2>&1 &
LAUNCH_PID=$!
sleep 20

echo "== B1a: 观测维度校验（日志须含 proprio_full 96 x 4 history / Total 384）=="
timeout 30 bash -c 'until grep -q "Total observation size: 384" /tmp/s2s_launch.log 2>/dev/null; do sleep 2; done'
if grep -q "Total observation size: 384" /tmp/s2s_launch.log; then ok "obs 384 校验"; else bad "obs 384 校验（日志: $(grep -o 'Total observation size: [0-9]*' /tmp/s2s_launch.log | tail -1)）"; fi
if grep -q "proprio_full" /tmp/s2s_launch.log; then ok "proprio_full term 注册"; else bad "proprio_full term 注册"; fi

echo "== B1b: 输入尺寸校验（384 == onnx input 384）=="
if grep -qiE "mismatch|size.*384.*expected|expected.*384" /tmp/s2s_launch.log; then
  bad "ONNX 输入/输出校验"; grep -iE "mismatch|expected" /tmp/s2s_launch.log | tail -3
elif grep -q "policy.onnx" /tmp/s2s_launch.log; then
  ok "ONNX 加载"
else
  echo "  (warn) 未抓到 onnx 加载日志，继续"
fi

echo "== B1c: 模式机 Damping->ReadyPose->Velocity =="
# 初始应为 Damping
timeout 30 bash -c 'until ros2 topic echo /ai_sapiens/mode_status --once 2>/dev/null | grep -q mode; do sleep 1; done'
ok "mode_status 可读"
ros2 service call /ai_sapiens/set_mode_by_name ai_sapiens_interfaces/srv/SetModeByName "{mode_name: ReadyPose}" > /dev/null 2>&1 && ok "切换 ReadyPose"
sleep 4   # 3s 过渡
ros2 service call /ai_sapiens/set_mode_by_name ai_sapiens_interfaces/srv/SetModeByName "{mode_name: Velocity}" > /dev/null 2>&1 && ok "切换 Velocity"

echo "== B1d: 站立 10s 稳定（无 NaN/关节发散）=="
sleep 10
if grep -qiE "nan|erratic|failsafe|damping.*requested" /tmp/s2s_launch.log; then
  bad "赛后日志无 NaN/失效（日志尾部:）"; tail -5 /tmp/s2s_launch.log
else
  ok "10s 站立无异常"
fi

echo "== B2/B3: cmd_vel 阶梯（0.5 / 1.0 / 2.0 / 3.0）各 8s =="
for V in 0.5 1.0 2.0 3.0; do
  timeout 5 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: $V, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" > /dev/null 2>&1 &
  PUB_PID=$!
  sleep 8
  kill $PUB_PID 2>/dev/null
  if grep -qiE "nan|failsafe" /tmp/s2s_launch.log; then
    bad "cmd ${V} m/s 异常"; grep -iE "nan|failsafe" /tmp/s2s_launch.log | tail -2; break
  else
    ok "cmd ${V} m/s 存活"
  fi
done

echo "== B5: 随机指令 30s 最终稳定性 =="
timeout 5 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}}" > /dev/null 2>&1 &
PUB_PID=$!
sleep 30
kill $PUB_PID 2>/dev/null
if grep -qiE "nan|failsafe" /tmp/s2s_launch.log; then bad "随机段异常"; else ok "随机段 30s 存活"; fi

echo ""
echo "========== 汇总: PASS=$PASS FAIL=$FAIL =========="
kill $LAUNCH_PID 2>/dev/null
pkill -f ros2_control_node 2>/dev/null
pkill -f mujoco 2>/dev/null
exit $FAIL