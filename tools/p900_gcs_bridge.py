#!/usr/bin/env python3
"""
地面站侧桥接：Mission Planner <-> 本机串口 <-> P900（与长机 swarm_node 的成帧协议一致）。

长机伴机期望收到：src=0xFE(GCS)、dst=长机 sysid、payload=原始 MAVLink 字节。
发往 MP：从电台读到 src=长机、dst=0xFE 的帧，去掉外层后写入 MP 串口。

用法示例:
  python3 p900_gcs_bridge.py --mp /dev/ttyUSB0 --radio /dev/ttyUSB1 --leader-sysid 1 --baud 57600

依赖: pip install pyserial
"""
import argparse
import struct
import sys

try:
    import serial
except ImportError:
    print("需要: pip install pyserial", file=sys.stderr)
    sys.exit(1)

MAGIC0, MAGIC1, VER = 0xAC, 0x9D, 1
ADDR_GCS = 0xFE


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def pack_frame(src: int, dst: int, payload: bytes) -> bytes:
    hdr = bytes([MAGIC0, MAGIC1, VER, src & 0xFF, dst & 0xFF])
    ln = len(payload)
    hdr += struct.pack(">H", ln)
    body = hdr + payload
    c = crc16_ccitt_false(body)
    return body + struct.pack(">H", c)


def try_unpack_one(buf: bytearray):
    """从 buf 开头解析一帧；成功则返回 (frame_bytes, payload, src, dst)，并截断 buf。"""
    need = 7 + 2
    if len(buf) < need:
        return None
    i = 0
    while i + 1 < len(buf) and not (buf[i] == MAGIC0 and buf[i + 1] == MAGIC1):
        i += 1
    if i > 0:
        del buf[:i]
    if len(buf) < need:
        return None
    if buf[2] != VER:
        del buf[:1]
        return None
    src, dst = buf[3], buf[4]
    plen = struct.unpack(">H", bytes(buf[5:7]))[0]
    if plen > 1024:
        del buf[:1]
        return None
    if len(buf) < 7 + plen + 2:
        return None
    body = bytes(buf[: 7 + plen])
    crc_got = struct.unpack(">H", bytes(buf[7 + plen : 9 + plen]))[0]
    if crc16_ccitt_false(body) != crc_got:
        del buf[:1]
        return None
    payload = bytes(buf[7 : 7 + plen])
    del buf[: 9 + plen]
    return (src, dst, payload)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mp", required=True, help="连接 Mission Planner 的串口")
    ap.add_argument("--radio", required=True, help="连接 P900(地面端)的串口")
    ap.add_argument("--leader-sysid", type=int, default=1)
    ap.add_argument("--baud", type=int, default=57600)
    args = ap.parse_args()

    mp = serial.Serial(args.mp, args.baud, timeout=0.01)
    radio = serial.Serial(args.radio, args.baud, timeout=0.01)
    leader = args.leader_sysid & 0xFF

    rx_radio = bytearray()
    mp_stream = bytearray()
    print("bridge running: MP<->framed P900, leader_sysid=%d" % leader, flush=True)

    def pop_mavlink_packets(stream: bytearray):
        """按 MAVLink v1/v2 整包拆包，避免半包进帧。"""
        out = []
        while len(stream) >= 2:
            if stream[0] == 0xFE and len(stream) >= 8:
                plen = stream[1]
                total = 8 + plen
                if len(stream) < total:
                    break
                out.append(bytes(stream[:total]))
                del stream[:total]
                continue
            if stream[0] == 0xFD and len(stream) >= 10:
                plen = stream[1]
                incompat = stream[2]
                total = 10 + plen + 2
                if incompat & 0x01:
                    total += 13
                if len(stream) < total:
                    break
                out.append(bytes(stream[:total]))
                del stream[:total]
                continue
            del stream[0]
        return out

    while True:
        # MP -> 按 MAVLink v2 整包成帧 -> 电台
        n = mp.in_waiting
        if n > 0:
            mp_stream.extend(mp.read(n))
        for pkt in pop_mavlink_packets(mp_stream):
            radio.write(pack_frame(ADDR_GCS, leader, pkt))

        # 电台 -> 解帧 -> MP（仅转发 dst==GCS 且来自长机）
        n = radio.in_waiting
        if n > 0:
            rx_radio.extend(radio.read(n))
        while True:
            r = try_unpack_one(rx_radio)
            if r is None:
                break
            src, dst, payload = r
            if dst == ADDR_GCS:
                mp.write(payload)


if __name__ == "__main__":
    main()
