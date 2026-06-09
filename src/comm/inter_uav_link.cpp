#include "comm/inter_uav_link.h"
#include "comm/link_mavlink_codec.h"
#include <chrono>
#include <cstdio>
#include <cstring>

bool InterUavLink::bind(const std::string &ip, int port) {
    return sock_.bind_local(ip, port);
}

void InterUavLink::set_target(const std::string &ip, int port) {
    target_ip_   = ip;
    target_port_ = port;
    sock_.set_target(ip, port);
}

void InterUavLink::set_local_identity(uint8_t sysid, uint8_t compid) {
    local_sysid_ = sysid;
    local_compid_ = compid;
}

void InterUavLink::note_peer(uint8_t sysid, const std::string &ip, int port) {
    if (sysid == 0 || ip.empty() || port <= 0)
        return;

    std::lock_guard<std::mutex> lk(peer_mtx_);
    const auto it = peers_.find(sysid);
    const bool changed = (it == peers_.end() || it->second.ip != ip || it->second.port != port);
    peers_[sysid] = PeerEndpoint{ip, port};
    if (changed) {
        printf("[udp-link] peer sysid=%u -> %s:%d\n",
               static_cast<unsigned>(sysid), ip.c_str(), port);
    }
}

int InterUavLink::send_packet(const LinkPacket &pkt) {
    uint8_t wire[MAVLINK_MAX_PACKET_LEN];
    int n = swarm_link_encode_packet(pkt, local_sysid_, local_compid_, wire, sizeof(wire));
    if (n <= 0) return -1;

    const size_t wire_len = static_cast<size_t>(n);

    if (pkt.dst_id != 0) {
        std::lock_guard<std::mutex> lk(peer_mtx_);
        const auto it = peers_.find(pkt.dst_id);
        if (it != peers_.end() && !it->second.ip.empty() && it->second.port > 0)
            return sock_.send_to(wire, wire_len, it->second.ip, it->second.port);
    }

    if (!target_ip_.empty() && target_port_ > 0)
        return sock_.send_to(wire, wire_len, target_ip_, target_port_);

    if (pkt.dst_id != 0) {
        static uint64_t last_warn_ms = 0;
        const uint64_t now_ms = []() {
            using namespace std::chrono;
            return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
        }();
        if (now_ms - last_warn_ms >= 5000) {
            last_warn_ms = now_ms;
            fprintf(stderr,
                    "[udp-link] no route to dst sysid=%u (await STATE_REPORT or check VPN)\n",
                    static_cast<unsigned>(pkt.dst_id));
        }
    }
    return -1;
}

bool InterUavLink::recv_packet(LinkPacket &pkt, int timeout_ms) {
    uint8_t buf[512];
    std::string from_ip;
    int from_port = 0;
    int n = sock_.recv_from(buf, sizeof(buf), timeout_ms, &from_ip, &from_port);
    if (n <= 0) return false;

    for (int i = 0; i < n; i++) {
        mavlink_message_t msg;
        if (!mavlink_parse_char(MAVLINK_COMM_1, buf[i], &msg, &mav_status_))
            continue;
        if (!swarm_link_decode_mavlink_msg(msg, pkt))
            continue;

        if (pkt.src_id != 0)
            note_peer(pkt.src_id, from_ip, from_port);
        return true;
    }
    return false;
}
