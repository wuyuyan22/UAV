#ifndef SWARM_P900_FRAMED_LINK_H
#define SWARM_P900_FRAMED_LINK_H

#include "comm/inter_uav_link.h"
#include "hal/serial_port.h"

#include <deque>
#include <mutex>
#include <vector>
#include <cstdint>
#include <string>
#include <unordered_map>

/** P900 透传 + 应用层成帧：单串口上区分「地面站」与「机间」逻辑通道。
 *  地面站侧需在 PC 上使用桥接工具将 MAVLink 包装为本帧格式（见 tools/p900_gcs_bridge.py）。 */
class P900FramedLink : public ISwarmAirLink {
public:
    /** 帧内地址：地面站（MP 桥接端） */
    static constexpr uint8_t ADDR_GCS = 0xFE;

    P900FramedLink() = default;
    ~P900FramedLink() override;

    bool open(const std::string &device, int baudrate);
    void close();

    bool bind(const std::string &ip, int port) override;
    void set_target(const std::string &ip, int port) override;
    void set_local_identity(uint8_t sysid, uint8_t compid = MAV_COMP_ID_ONBOARD_COMPUTER) override;
    int  send_packet(const LinkPacket &pkt) override;
    bool recv_packet(LinkPacket &pkt, int timeout_ms = 5) override;

    /** 从串口读字节并解析帧，填充内部队列（在收包线程中高频调用） */
    void rx_pump();

    /** 长机：取出一字节来自 GCS 通道的 MAVLink 流（无数据返回 false） */
    bool pop_gcs_byte(uint8_t *out);

    /** 长机：将 MAVLink 线字节发往地面站通道（src=本机, dst=ADDR_GCS） */
    int write_mavlink_to_gcs(const uint8_t *wire, uint16_t len);

    bool is_open() const { return serial_.is_open(); }

private:
    static uint16_t crc16_ccitt_false(const uint8_t *data, size_t len);

    bool try_parse_one_frame();
    int  write_frame(uint8_t src, uint8_t dst, const uint8_t *payload, uint16_t len);

    SerialPort serial_;
    std::mutex tx_mtx_;

    uint8_t local_sysid_  = 1;
    uint8_t local_compid_ = MAV_COMP_ID_ONBOARD_COMPUTER;

    std::vector<uint8_t> rx_raw_;
    std::deque<LinkPacket> swarm_rx_;
    std::deque<uint8_t>    gcs_rx_;
    std::mutex             q_mtx_;

    static constexpr size_t kMaxGcsRx = 65536;

    struct DiagCounters {
        uint64_t tx_bytes = 0;
        uint64_t tx_frame_ok = 0;
        uint64_t tx_frame_fail = 0;
        uint64_t rx_bytes = 0;
        uint64_t frame_ok = 0;
        uint64_t frame_crc_err = 0;
        uint64_t frame_ver_err = 0;
        uint64_t frame_len_err = 0;
        uint64_t frame_drop_not_for_me = 0;
        uint64_t frame_gcs_to_me = 0;
        uint64_t frame_swarm_to_me = 0;
        uint64_t lp_decode_ok = 0;
        uint64_t lp_decode_fail = 0;
    };

    DiagCounters diag_{};
    std::unordered_map<uint8_t, uint64_t> diag_tx_dst_cnt_;
    std::unordered_map<uint8_t, uint64_t> diag_tx_type_cnt_;
    std::unordered_map<uint8_t, uint64_t> diag_lp_type_cnt_;
    uint64_t diag_last_print_ms_ = 0;
    uint8_t diag_last_frame_src_ = 0;
    uint8_t diag_last_frame_dst_ = 0;
    uint8_t diag_last_lp_src_ = 0;
    uint8_t diag_last_lp_dst_ = 0;
    uint8_t diag_last_lp_type_ = 0;
    uint8_t diag_last_tx_src_ = 0;
    uint8_t diag_last_tx_dst_ = 0;
    uint16_t diag_last_tx_len_ = 0;
    uint8_t diag_last_tx_type_ = 0xFF;
};

#endif
