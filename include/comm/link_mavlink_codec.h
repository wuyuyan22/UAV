#ifndef SWARM_LINK_MAVLINK_CODEC_H
#define SWARM_LINK_MAVLINK_CODEC_H

#include "comm/inter_uav_link.h"
#include <cstddef>
#include <cstdint>

#include <ardupilotmega/mavlink.h>

/** LinkPacket <-> 标准 MAVLink 线字节（与 UDP/P900 传输无关） */
int swarm_link_encode_packet(const LinkPacket &pkt,
                             uint8_t local_sysid,
                             uint8_t local_compid,
                             uint8_t *wire_out,
                             size_t wire_max);

bool swarm_link_decode_mavlink_msg(const mavlink_message_t &msg, LinkPacket &pkt);

#endif
