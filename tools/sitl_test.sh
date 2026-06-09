#!/bin/bash
# ============================================================
# SITL 多机联调脚本
# 使用 ArduPilot SITL 模拟器测试群控程序
# 前提：已安装 ardupilot 开发环境
# ============================================================

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
BUILD_DIR="$(dirname "$0")/../build"

echo "=== UAV Swarm SITL Test ==="

# 1. 启动 3 个 SITL 实例（sysid=1,2,3）
echo "[1/3] Starting SITL instances..."

cd "$ARDUPILOT_DIR"

# Leader (sysid=1, UDP 14550)
sim_vehicle.py -v ArduPlane -I 0 --sysid 1 \
    --out "udp:127.0.0.1:14550" \
    -l "37.339524,117.933845,100,0" \
    --no-mavproxy &
SITL_PID1=$!

sleep 3

# Follower 1 (sysid=2, UDP 14560)
sim_vehicle.py -v ArduPlane -I 1 --sysid 2 \
    --out "udp:127.0.0.1:14560" \
    -l "37.339524,117.923845,100,0" \
    --no-mavproxy &
SITL_PID2=$!

sleep 3

# Follower 2 (sysid=3, UDP 14570)
sim_vehicle.py -v ArduPlane -I 2 --sysid 3 \
    --out "udp:127.0.0.1:14570" \
    -l "37.339524,117.943845,100,0" \
    --no-mavproxy &
SITL_PID3=$!

sleep 5

# 2. 启动群控程序
echo "[2/3] Starting swarm controller..."

if [ ! -f "$BUILD_DIR/swarm_node" ]; then
    echo "Error: swarm_node not found. Build first:"
    echo "  cd build && cmake .. && make"
    kill $SITL_PID1 $SITL_PID2 $SITL_PID3 2>/dev/null
    exit 1
fi

# Leader 节点
"$BUILD_DIR/swarm_node" 1 1 --udp 127.0.0.1:14550 --link-port 19870 --hz 10 &
SWARM_PID1=$!

sleep 2

# Follower 1
"$BUILD_DIR/swarm_node" 2 2 --udp 127.0.0.1:14560 --link-port 19871 --leader-ip 127.0.0.1 --hz 10 &
SWARM_PID2=$!

# Follower 2
"$BUILD_DIR/swarm_node" 3 2 --udp 127.0.0.1:14570 --link-port 19872 --leader-ip 127.0.0.1 --hz 10 &
SWARM_PID3=$!

echo "[3/3] All processes started."
echo "  SITL PIDs : $SITL_PID1 $SITL_PID2 $SITL_PID3"
echo "  Swarm PIDs: $SWARM_PID1 $SWARM_PID2 $SWARM_PID3"
echo ""
echo "Press Ctrl+C to stop all."

cleanup() {
    echo "Stopping..."
    kill $SWARM_PID1 $SWARM_PID2 $SWARM_PID3 2>/dev/null
    kill $SITL_PID1 $SITL_PID2 $SITL_PID3 2>/dev/null
    wait
    echo "Done."
}

trap cleanup SIGINT SIGTERM
wait
