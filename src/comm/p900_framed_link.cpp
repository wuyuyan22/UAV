#include "comm/p900_framed_link.h"
#include "comm/link_mavlink_codec.h"
#include <cstring>
#include <cstdio>
#include <chrono>

namespace {

constexpr uint8_t kMagic0 = 0xAC;
constexpr uint8_t kMagic1 = 0x9D;
constexpr uint8_t kVer    = 1;

uint64_t steady_ms_now() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

}  // namespace

P900FramedLink::~P900FramedLink() { close(); }

uint16_t P900FramedLink::crc16_ccitt_false(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000)
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            else
                crc = static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

bool P900FramedLink::open(const std::string &device, int baudrate) {
    close();
    return serial_.open(device, baudrate);
}

void P900FramedLink::close() {
    std::lock_guard<std::mutex> lk(q_mtx_);
    rx_raw_.clear();
    swarm_rx_.clear();
    gcs_rx_.clear();
    diag_ = DiagCounters{};
    diag_tx_dst_cnt_.clear();
    diag_tx_type_cnt_.clear();
    diag_lp_type_cnt_.clear();
    diag_last_print_ms_ = 0;
    diag_last_frame_src_ = 0;
    diag_last_frame_dst_ = 0;
    diag_last_lp_src_ = 0;
    diag_last_lp_dst_ = 0;
    diag_last_lp_type_ = 0;
    diag_last_tx_src_ = 0;
    diag_last_tx_dst_ = 0;
    diag_last_tx_len_ = 0;
    diag_last_tx_type_ = 0xFF;
    serial_.close();
}

bool P900FramedLink::bind(const std::string &, int) { return serial_.is_open(); }

void P900FramedLink::set_target(const std::string &, int) {}

void P900FramedLink::set_local_identity(uint8_t sysid, uint8_t compid) {
    local_sysid_  = sysid;
    local_compid_ = compid;
}

int P900FramedLink::write_frame(uint8_t src, uint8_t dst, const uint8_t *payload, uint16_t len) {
    if (!serial_.is_open()) return -1;
    if (len > 1024) return -1;

    uint8_t hdr[7];
    hdr[0] = kMagic0;
    hdr[1] = kMagic1;
    hdr[2] = kVer;
    hdr[3] = src;
    hdr[4] = dst;
    hdr[5] = static_cast<uint8_t>((len >> 8) & 0xFF);
    hdr[6] = static_cast<uint8_t>(len & 0xFF);

    uint8_t crcbuf[7 + 1024];
    memcpy(crcbuf, hdr, 7);
    memcpy(crcbuf + 7, payload, len);
    uint16_t crc = crc16_ccitt_false(crcbuf, 7 + len);

    uint8_t tail[2] = {static_cast<uint8_t>(crc >> 8), static_cast<uint8_t>(crc & 0xFF)};

    std::lock_guard<std::mutex> lk(tx_mtx_);
    if (serial_.write_bytes(hdr, 7) != 7) {
        diag_.tx_frame_fail++;
        return -1;
    }
    if (len > 0 && static_cast<size_t>(serial_.write_bytes(payload, len)) != len) {
        diag_.tx_frame_fail++;
        return -1;
    }
    if (serial_.write_bytes(tail, 2) != 2) {
        diag_.tx_frame_fail++;
        return -1;
    }
    diag_.tx_frame_ok++;
    diag_.tx_bytes += static_cast<uint64_t>(7 + len + 2);
    diag_tx_dst_cnt_[dst]++;
    diag_last_tx_src_ = src;
    diag_last_tx_dst_ = dst;
    diag_last_tx_len_ = len;
    return 7 + len + 2;
}

int P900FramedLink::send_packet(const LinkPacket &pkt) {
    uint8_t wire[MAVLINK_MAX_PACKET_LEN];
    int n = swarm_link_encode_packet(pkt, local_sysid_, local_compid_, wire, sizeof(wire));
    if (n <= 0) return -1;

    const uint8_t dst = static_cast<uint8_t>(pkt.dst_id);
    diag_tx_type_cnt_[static_cast<uint8_t>(pkt.msg_type)]++;
    diag_last_tx_type_ = static_cast<uint8_t>(pkt.msg_type);
    return write_frame(local_sysid_, dst, wire, static_cast<uint16_t>(n));
}

int P900FramedLink::write_mavlink_to_gcs(const uint8_t *wire, uint16_t len) {
    diag_last_tx_type_ = 0xFF;  // 非 LinkPacket 业务帧
    return write_frame(local_sysid_, ADDR_GCS, wire, len);
}

bool P900FramedLink::try_parse_one_frame() {
    const size_t need = 7 + 2;
    if (rx_raw_.size() < need) return false;

    size_t i = 0;
    for (; i + 1 < rx_raw_.size(); i++) {
        if (rx_raw_[i] == kMagic0 && rx_raw_[i + 1] == kMagic1) break;
    }
    if (i > 0) rx_raw_.erase(rx_raw_.begin(), rx_raw_.begin() + static_cast<std::ptrdiff_t>(i));
    if (rx_raw_.size() < need) return false;

    if (rx_raw_[0] != kMagic0 || rx_raw_[1] != kMagic1) return false;
    if (rx_raw_[2] != kVer) {
        diag_.frame_ver_err++;
        rx_raw_.erase(rx_raw_.begin());
        return false;
    }

    uint8_t src = rx_raw_[3];
    uint8_t dst = rx_raw_[4];
    diag_last_frame_src_ = src;
    diag_last_frame_dst_ = dst;
    uint16_t plen = static_cast<uint16_t>((rx_raw_[5] << 8) | rx_raw_[6]);
    if (plen > 1024) {
        diag_.frame_len_err++;
        rx_raw_.erase(rx_raw_.begin());
        return false;
    }
    if (rx_raw_.size() < 7 + plen + 2) return false;

    uint8_t crcbuf[7 + 1024];
    memcpy(crcbuf, rx_raw_.data(), 7 + plen);
    uint16_t crc_exp = crc16_ccitt_false(crcbuf, 7 + plen);
    uint16_t crc_got = static_cast<uint16_t>((rx_raw_[7 + plen] << 8) | rx_raw_[7 + plen + 1]);
    if (crc_exp != crc_got) {
        diag_.frame_crc_err++;
        fprintf(stderr, "[p900] CRC mismatch, drop sync (src=%u dst=%u len=%u)\n",
                static_cast<unsigned>(src),
                static_cast<unsigned>(dst),
                static_cast<unsigned>(plen));
        rx_raw_.erase(rx_raw_.begin());
        return false;
    }
    diag_.frame_ok++;

    const uint8_t *payload = rx_raw_.data() + 7;
    rx_raw_.erase(rx_raw_.begin(), rx_raw_.begin() + static_cast<std::ptrdiff_t>(7 + plen + 2));

    std::lock_guard<std::mutex> lk(q_mtx_);

    if (src == ADDR_GCS && dst == local_sysid_) {
        diag_.frame_gcs_to_me++;
        for (uint16_t k = 0; k < plen; k++) {
            gcs_rx_.push_back(payload[k]);
            if (gcs_rx_.size() > kMaxGcsRx)
                gcs_rx_.pop_front();
        }
        return true;
    }

    if (dst != local_sysid_ && dst != 0) {
        diag_.frame_drop_not_for_me++;
        return true;
    }
    diag_.frame_swarm_to_me++;

    mavlink_message_t msg{};
    mavlink_status_t st{};
    for (uint16_t k = 0; k < plen; k++) {
        if (mavlink_parse_char(MAVLINK_COMM_2, payload[k], &msg, &st)) {
            LinkPacket lp{};
            if (swarm_link_decode_mavlink_msg(msg, lp)) {
                swarm_rx_.push_back(lp);
                diag_.lp_decode_ok++;
                diag_lp_type_cnt_[static_cast<uint8_t>(lp.msg_type)]++;
                diag_last_lp_src_ = lp.src_id;
                diag_last_lp_dst_ = lp.dst_id;
                diag_last_lp_type_ = static_cast<uint8_t>(lp.msg_type);
            } else {
                diag_.lp_decode_fail++;
            }
        }
    }
    return true;
}

void P900FramedLink::rx_pump() {
    if (!serial_.is_open()) return;
    uint8_t buf[512];
    const uint64_t now0 = steady_ms_now();
    if (diag_last_print_ms_ == 0)
        diag_last_print_ms_ = now0;
    for (;;) {
        int n = serial_.read_bytes(buf, sizeof(buf));
        if (n <= 0) break;
        diag_.rx_bytes += static_cast<uint64_t>(n);
        rx_raw_.insert(rx_raw_.end(), buf, buf + n);
        while (try_parse_one_frame()) {}
        if (rx_raw_.size() > 8192) {
            fprintf(stderr, "[p900] rx_raw overflow, clearing\n");
            rx_raw_.clear();
        }
    }

    const uint64_t now = steady_ms_now();
    constexpr uint64_t kDiagPrintIntervalMs = 5000;
    if (now - diag_last_print_ms_ >= kDiagPrintIntervalMs) {
        const bool has_activity =
            (diag_.tx_frame_ok + diag_.tx_frame_fail + diag_.rx_bytes +
             diag_.frame_ok + diag_.frame_crc_err + diag_.frame_ver_err +
             diag_.frame_len_err + diag_.frame_drop_not_for_me +
             diag_.frame_gcs_to_me + diag_.frame_swarm_to_me +
             diag_.lp_decode_ok + diag_.lp_decode_fail) > 0;
        if (!has_activity) {
            diag_last_print_ms_ = now;
            return;
        }
        fprintf(stderr,
                "[p900][diag] tx_ok=%llu tx_fail=%llu tx_bytes=%llu rx_bytes=%llu "
                "ok=%llu crc_err=%llu ver_err=%llu len_err=%llu drop_dst=%llu "
                "gcs=%llu swarm=%llu lp_ok=%llu lp_fail=%llu "
                "last_tx=%u->%u len=%u tx_type=0x%02X last_frame=%u->%u last_lp=%u->%u type=0x%02X",
                static_cast<unsigned long long>(diag_.tx_frame_ok),
                static_cast<unsigned long long>(diag_.tx_frame_fail),
                static_cast<unsigned long long>(diag_.tx_bytes),
                static_cast<unsigned long long>(diag_.rx_bytes),
                static_cast<unsigned long long>(diag_.frame_ok),
                static_cast<unsigned long long>(diag_.frame_crc_err),
                static_cast<unsigned long long>(diag_.frame_ver_err),
                static_cast<unsigned long long>(diag_.frame_len_err),
                static_cast<unsigned long long>(diag_.frame_drop_not_for_me),
                static_cast<unsigned long long>(diag_.frame_gcs_to_me),
                static_cast<unsigned long long>(diag_.frame_swarm_to_me),
                static_cast<unsigned long long>(diag_.lp_decode_ok),
                static_cast<unsigned long long>(diag_.lp_decode_fail),
                static_cast<unsigned>(diag_last_tx_src_),
                static_cast<unsigned>(diag_last_tx_dst_),
                static_cast<unsigned>(diag_last_tx_len_),
                static_cast<unsigned>(diag_last_tx_type_),
                static_cast<unsigned>(diag_last_frame_src_),
                static_cast<unsigned>(diag_last_frame_dst_),
                static_cast<unsigned>(diag_last_lp_src_),
                static_cast<unsigned>(diag_last_lp_dst_),
                static_cast<unsigned>(diag_last_lp_type_));
        if (!diag_tx_dst_cnt_.empty()) {
            fprintf(stderr, " tx_dst=");
            bool first = true;
            for (const auto &kv : diag_tx_dst_cnt_) {
                if (!first) fprintf(stderr, ",");
                first = false;
                fprintf(stderr, "%u:%llu",
                        static_cast<unsigned>(kv.first),
                        static_cast<unsigned long long>(kv.second));
            }
        }
        if (!diag_tx_type_cnt_.empty()) {
            fprintf(stderr, " tx_types=");
            bool first = true;
            for (const auto &kv : diag_tx_type_cnt_) {
                if (!first) fprintf(stderr, ",");
                first = false;
                fprintf(stderr, "0x%02X:%llu",
                        static_cast<unsigned>(kv.first),
                        static_cast<unsigned long long>(kv.second));
            }
        }
        if (!diag_lp_type_cnt_.empty()) {
            fprintf(stderr, " lp_types=");
            bool first = true;
            for (const auto &kv : diag_lp_type_cnt_) {
                if (!first) fprintf(stderr, ",");
                first = false;
                fprintf(stderr, "0x%02X:%llu",
                        static_cast<unsigned>(kv.first),
                        static_cast<unsigned long long>(kv.second));
            }
        }
        fprintf(stderr, "\n");
        diag_ = DiagCounters{};
        diag_tx_dst_cnt_.clear();
        diag_tx_type_cnt_.clear();
        diag_lp_type_cnt_.clear();
        diag_last_print_ms_ = now;
    }
}

bool P900FramedLink::recv_packet(LinkPacket &pkt, int) {
    std::lock_guard<std::mutex> lk(q_mtx_);
    if (swarm_rx_.empty()) return false;
    pkt = swarm_rx_.front();
    swarm_rx_.pop_front();
    return true;
}

bool P900FramedLink::pop_gcs_byte(uint8_t *out) {
    std::lock_guard<std::mutex> lk(q_mtx_);
    if (gcs_rx_.empty()) return false;
    *out = gcs_rx_.front();
    gcs_rx_.pop_front();
    return true;
}
