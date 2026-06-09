#include "comm/link_mavlink_codec.h"
#include "common/types.h"
#include <cmath>

int swarm_link_encode_packet(const LinkPacket &pkt,
                             uint8_t local_sysid,
                             uint8_t local_compid,
                             uint8_t *wire_out,
                             size_t wire_max) {
    mavlink_message_t msg{};

    if (pkt.msg_type == LinkMsgType::STATE_REPORT) {
        mavlink_global_position_int_t gp{};
        gp.time_boot_ms = pkt.custom_mode;
        gp.lat = static_cast<int32_t>(pkt.lat * SWARM_RAD2DEG * 1e7);
        gp.lon = static_cast<int32_t>(pkt.lon * SWARM_RAD2DEG * 1e7);
        gp.relative_alt = static_cast<int32_t>(pkt.alt * 1000.0f);
        // 用 alt=1000/1001 作为新协议标记并携带 armed，旧协议该值通常为 0。
        gp.alt = pkt.armed ? 1001 : 1000;
        float vn = pkt.vel * cosf(pkt.hdg);
        float ve = pkt.vel * sinf(pkt.hdg);
        gp.vx = static_cast<int16_t>(vn * 100.0f);
        gp.vy = static_cast<int16_t>(ve * 100.0f);
        gp.vz = 0;
        float hdg_deg = pkt.hdg * SWARM_RAD2DEG;
        if (hdg_deg < 0) hdg_deg += 360.0f;
        if (hdg_deg >= 360.0f) hdg_deg -= 360.0f;
        gp.hdg = static_cast<uint16_t>(hdg_deg * 100.0f);
        mavlink_msg_global_position_int_encode(local_sysid, local_compid, &msg, &gp);
    } else if (pkt.msg_type == LinkMsgType::FOLLOWER_CMD) {
        mavlink_set_position_target_global_int_t sp{};
        sp.time_boot_ms = 0;
        sp.coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
        sp.type_mask = TypeMask::USE_POS_AND_VEL;
        sp.lat_int = static_cast<int32_t>(pkt.lat * SWARM_RAD2DEG * 1e7);
        sp.lon_int = static_cast<int32_t>(pkt.lon * SWARM_RAD2DEG * 1e7);
        sp.alt = pkt.alt;
        sp.vx = pkt.vel * cosf(pkt.hdg);
        sp.vy = pkt.vel * sinf(pkt.hdg);
        sp.vz = 0;
        sp.yaw = pkt.hdg;
        sp.yaw_rate = 0;
        sp.target_system = static_cast<uint8_t>(pkt.dst_id);
        sp.target_component = MAV_COMP_ID_AUTOPILOT1;
        mavlink_msg_set_position_target_global_int_encode(local_sysid, local_compid, &msg, &sp);
    } else if (pkt.msg_type == LinkMsgType::TASK_META ||
               pkt.msg_type == LinkMsgType::TASK_COMMIT ||
               pkt.msg_type == LinkMsgType::TASK_ACK ||
               pkt.msg_type == LinkMsgType::TASK_START) {
        mavlink_command_long_t cl{};
        cl.target_system = static_cast<uint8_t>(pkt.dst_id);
        cl.target_component = MAV_COMP_ID_ONBOARD_COMPUTER;
        cl.param1 = static_cast<float>(pkt.seq);
        cl.param2 = pkt.alt;
        if (pkt.msg_type == LinkMsgType::TASK_META) cl.command = SwarmMavCmd::TASK_BEGIN;
        if (pkt.msg_type == LinkMsgType::TASK_COMMIT) cl.command = SwarmMavCmd::TASK_COMMIT;
        if (pkt.msg_type == LinkMsgType::TASK_ACK) cl.command = SwarmMavCmd::TASK_ACK;
        if (pkt.msg_type == LinkMsgType::TASK_START) cl.command = SwarmMavCmd::TASK_START_AUTO;
        mavlink_msg_command_long_encode(local_sysid, local_compid, &msg, &cl);
    } else if (pkt.msg_type == LinkMsgType::TASK_ITEM) {
        mavlink_mission_item_int_t it{};
        it.target_system = static_cast<uint8_t>(pkt.dst_id);
        it.target_component = MAV_COMP_ID_AUTOPILOT1;
        it.seq = static_cast<uint16_t>(pkt.seq);
        it.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
        it.command = MAV_CMD_NAV_WAYPOINT;
        it.current = (pkt.seq == 0) ? 1 : 0;
        it.autocontinue = 1;
        it.x = static_cast<int32_t>(pkt.lat * SWARM_RAD2DEG * 1e7);
        it.y = static_cast<int32_t>(pkt.lon * SWARM_RAD2DEG * 1e7);
        it.z = pkt.alt;
        it.mission_type = MAV_MISSION_TYPE_MISSION;
        mavlink_msg_mission_item_int_encode(local_sysid, local_compid, &msg, &it);
    } else if (pkt.msg_type == LinkMsgType::CTRL_CMD) {
        mavlink_command_long_t cl{};
        cl.target_system = static_cast<uint8_t>(pkt.dst_id);
        cl.target_component = MAV_COMP_ID_AUTOPILOT1;
        cl.command = SwarmMavCmd::CTRL_PROXY;
        cl.confirmation = 0;
        cl.param1 = static_cast<float>(pkt.seq);
        cl.param2 = pkt.ctrl_pf[0];
        cl.param3 = pkt.ctrl_pf[1];
        cl.param4 = pkt.ctrl_pf[2];
        cl.param5 = pkt.ctrl_pf[3];
        cl.param6 = pkt.ctrl_pf[4];
        cl.param7 = pkt.ctrl_pf[5];
        mavlink_msg_command_long_encode(local_sysid, local_compid, &msg, &cl);
    } else {
        return -1;
    }

    uint16_t wire_len = mavlink_msg_to_send_buffer(wire_out, &msg);
    if (wire_len > wire_max) return -1;
    return static_cast<int>(wire_len);
}

bool swarm_link_decode_mavlink_msg(const mavlink_message_t &msg, LinkPacket &pkt) {
    if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        mavlink_global_position_int_t gp{};
        mavlink_msg_global_position_int_decode(&msg, &gp);
        pkt.msg_type = LinkMsgType::STATE_REPORT;
        pkt.src_id = msg.sysid;
        pkt.dst_id = 0;
        pkt.lat = gp.lat * 1e-7 * SWARM_DEG2RAD;
        pkt.lon = gp.lon * 1e-7 * SWARM_DEG2RAD;
        pkt.alt = gp.relative_alt * 0.001f;
        float vn = gp.vx * 0.01f;
        float ve = gp.vy * 0.01f;
        pkt.vel = sqrtf(vn * vn + ve * ve);
        pkt.hdg = (gp.hdg == UINT16_MAX) ? 0.0f : (gp.hdg * 0.01f * SWARM_DEG2RAD);
        pkt.pitch = 0;
        pkt.roll = 0;
        pkt.health = 0xFFFF;
        if (gp.alt == 1000 || gp.alt == 1001) {
            pkt.mode_valid = 1;
            pkt.armed = (gp.alt == 1001) ? 1 : 0;
            pkt.custom_mode = gp.time_boot_ms;
        } else {
            pkt.mode_valid = 0;
            pkt.armed = 0;
            pkt.custom_mode = 0;
        }
        return true;
    }
    if (msg.msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT) {
        mavlink_set_position_target_global_int_t sp{};
        mavlink_msg_set_position_target_global_int_decode(&msg, &sp);
        pkt.msg_type = LinkMsgType::FOLLOWER_CMD;
        pkt.src_id = msg.sysid;
        pkt.dst_id = sp.target_system;
        pkt.lat = sp.lat_int * 1e-7 * SWARM_DEG2RAD;
        pkt.lon = sp.lon_int * 1e-7 * SWARM_DEG2RAD;
        pkt.alt = sp.alt;
        pkt.vel = sqrtf(sp.vx * sp.vx + sp.vy * sp.vy);
        pkt.hdg = sp.yaw;
        pkt.pitch = 0;
        pkt.roll = 0;
        pkt.health = 0xFFFF;
        return true;
    }
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        mavlink_command_long_t cl{};
        mavlink_msg_command_long_decode(&msg, &cl);
        if (cl.command == SwarmMavCmd::CTRL_PROXY) {
            pkt.msg_type = LinkMsgType::CTRL_CMD;
            pkt.src_id = msg.sysid;
            pkt.dst_id = cl.target_system;
            pkt.seq = static_cast<uint32_t>(cl.param1 + 0.5f);
            pkt.ctrl_pf[0] = cl.param2;
            pkt.ctrl_pf[1] = cl.param3;
            pkt.ctrl_pf[2] = cl.param4;
            pkt.ctrl_pf[3] = cl.param5;
            pkt.ctrl_pf[4] = cl.param6;
            pkt.ctrl_pf[5] = cl.param7;
            return true;
        }
        if (cl.command == SwarmMavCmd::TASK_BEGIN) {
            pkt.msg_type = LinkMsgType::TASK_META;
            pkt.src_id = msg.sysid;
            pkt.dst_id = cl.target_system;
            pkt.seq = static_cast<uint32_t>(cl.param1);
            pkt.alt = cl.param2;
            return true;
        }
        if (cl.command == SwarmMavCmd::TASK_COMMIT) {
            pkt.msg_type = LinkMsgType::TASK_COMMIT;
            pkt.src_id = msg.sysid;
            pkt.dst_id = cl.target_system;
            pkt.seq = static_cast<uint32_t>(cl.param1);
            return true;
        }
        if (cl.command == SwarmMavCmd::TASK_ACK) {
            pkt.msg_type = LinkMsgType::TASK_ACK;
            pkt.src_id = msg.sysid;
            pkt.dst_id = cl.target_system;
            pkt.seq = static_cast<uint32_t>(cl.param1);
            pkt.alt = cl.param2;
            return true;
        }
        if (cl.command == SwarmMavCmd::TASK_START_AUTO) {
            pkt.msg_type = LinkMsgType::TASK_START;
            pkt.src_id = msg.sysid;
            pkt.dst_id = cl.target_system;
            pkt.seq = static_cast<uint32_t>(cl.param1);
            return true;
        }
    }
    if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM_INT) {
        mavlink_mission_item_int_t it{};
        mavlink_msg_mission_item_int_decode(&msg, &it);
        pkt.msg_type = LinkMsgType::TASK_ITEM;
        pkt.src_id = msg.sysid;
        pkt.dst_id = it.target_system;
        pkt.seq = it.seq;
        pkt.lat = it.x * 1e-7 * SWARM_DEG2RAD;
        pkt.lon = it.y * 1e-7 * SWARM_DEG2RAD;
        pkt.alt = it.z;
        return true;
    }
    return false;
}
