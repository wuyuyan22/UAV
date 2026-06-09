#!/usr/bin/env bash
# 5G 数图传一体 — 伴飞主机出口 QoS（HTB + fq_codel + DSCP）
# 对应方案 §6.5：MAVLink 14550/UDP 高优先级、RTP 5000+ 次优先级、其他默认低优先级
#
# 用法：
#   swarm-qos.sh [IFACE] [LINK_KBIT] [TELEM_KBIT] [VIDEO_KBIT] [GCS_PORT] [RTP_PORT_BASE]
#   IFACE          : 出口接口（默认 wg0；裸 5G 可设 wwan0/usb0）
#   LINK_KBIT      : 链路上行总速率（kbit），按实测 × 0.85 设置
#   TELEM_KBIT     : MAVLink 占用预算（kbit）
#   VIDEO_KBIT    : 图传占用预算（kbit）
#   GCS_PORT       : GCS UDP 端口（默认 14550）
#   RTP_PORT_BASE  : 图传 UDP 端口段起点（默认 5000；匹配 5000..5015）
#
# 安全：脚本会先 `tc qdisc del root`，再重新 attach；幂等可重复运行。
set -euo pipefail

IFACE="${1:-wg0}"
LINK_KBIT="${2:-4000}"
TELEM_KBIT="${3:-512}"
VIDEO_KBIT="${4:-2500}"
GCS_PORT="${5:-14550}"
RTP_PORT_BASE="${6:-5000}"

if ! command -v tc >/dev/null 2>&1; then
    echo "[swarm-qos] tc not found, please install iproute2" >&2
    exit 1
fi

if ! ip link show "$IFACE" >/dev/null 2>&1; then
    echo "[swarm-qos] interface $IFACE not present, skipping" >&2
    exit 0
fi

# RK356x 等 OEM 内核常未编入 sch_htb / sch_fq_codel；探测失败则跳过（exit 0），避免拖垮 systemd
if ! tc qdisc add dev lo root handle 999: htb 2>/dev/null; then
    echo "[swarm-qos] HTB qdisc unavailable on kernel $(uname -r); QoS skipped (install sch_htb or use stock kernel)" >&2
    exit 0
fi
tc qdisc del dev lo root handle 999: 2>/dev/null || true

LEAF_QDISC="fq_codel"
if ! tc qdisc add dev lo root handle 998: fq_codel 2>/dev/null; then
    LEAF_QDISC="pfifo_fast"
fi
tc qdisc del dev lo root handle 998: 2>/dev/null || true

echo "[swarm-qos] iface=$IFACE link=${LINK_KBIT}kbit telem=${TELEM_KBIT}kbit video=${VIDEO_KBIT}kbit gcs_port=$GCS_PORT rtp_base=$RTP_PORT_BASE leaf=$LEAF_QDISC"

tc qdisc del dev "$IFACE" root 2>/dev/null || true

tc qdisc add dev "$IFACE" root handle 1: htb default 30 r2q 1

tc class add dev "$IFACE" parent 1:   classid 1:1  htb rate "${LINK_KBIT}kbit" ceil "${LINK_KBIT}kbit"
tc class add dev "$IFACE" parent 1:1  classid 1:10 htb rate "${TELEM_KBIT}kbit" ceil "${LINK_KBIT}kbit" prio 0
tc class add dev "$IFACE" parent 1:1  classid 1:20 htb rate "${VIDEO_KBIT}kbit" ceil "${LINK_KBIT}kbit" prio 1
tc class add dev "$IFACE" parent 1:1  classid 1:30 htb rate 64kbit              ceil "${LINK_KBIT}kbit" prio 7

if [[ "$LEAF_QDISC" == "fq_codel" ]]; then
    tc qdisc add dev "$IFACE" parent 1:10 handle 10: fq_codel target 5ms
    tc qdisc add dev "$IFACE" parent 1:20 handle 20: fq_codel target 20ms
    tc qdisc add dev "$IFACE" parent 1:30 handle 30: fq_codel
else
    tc qdisc add dev "$IFACE" parent 1:10 handle 10: pfifo_fast
    tc qdisc add dev "$IFACE" parent 1:20 handle 20: pfifo_fast
    tc qdisc add dev "$IFACE" parent 1:30 handle 30: pfifo_fast
fi

# MAVLink GCS（按目的与源端口同时打标，覆盖来回方向）
tc filter add dev "$IFACE" parent 1: protocol ip prio 1 u32 \
   match ip dport "$GCS_PORT" 0xffff flowid 1:10
tc filter add dev "$IFACE" parent 1: protocol ip prio 1 u32 \
   match ip sport "$GCS_PORT" 0xffff flowid 1:10

# 图传：5000..5015（端口掩码 0xfff0，匹配 5000+sysid 段）
tc filter add dev "$IFACE" parent 1: protocol ip prio 2 u32 \
   match ip dport "$RTP_PORT_BASE" 0xfff0 flowid 1:20

# DSCP AF41（伴飞应用层亦设 IP_TOS=0x88，运营商承载侧可识别）
tc filter add dev "$IFACE" parent 1: protocol ip prio 3 u32 \
   match ip tos 0x88 0xfc flowid 1:10

echo "[swarm-qos] applied:"
tc -s qdisc show dev "$IFACE"
