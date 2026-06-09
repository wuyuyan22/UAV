#ifndef SWARM_GCS_IP_LINK_H
#define SWARM_GCS_IP_LINK_H

#include "comm/gcs_link.h"

#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

class GcsIpLink : public IGcsLink {
public:
    /** NAT 保活心跳生成回调：实现者将 MAVLink HEARTBEAT 线字节写入 wire，返回字节数；
     *  返回 0 表示本 tick 不发心跳。 */
    using HeartbeatBuilder =
        std::function<uint16_t(uint8_t *wire, uint16_t max_len)>;

    struct Stats {
        uint64_t tx_packets = 0;
        uint64_t tx_bytes = 0;
        uint64_t rx_packets = 0;
        uint64_t rx_bytes = 0;
        uint64_t send_errors = 0;
        uint64_t last_send_err_ms = 0;
        uint64_t last_send_err_no = 0;
        uint64_t last_recv_ms = 0;
        uint64_t last_send_ms = 0;
        uint64_t last_heartbeat_tx_ms = 0;
    };

    GcsIpLink(const std::string &bind_ip,
              int bind_port,
              const std::string &target_ip,
              int target_port);
    ~GcsIpLink() override;

    bool open() override;
    void close() override;
    int  write_mavlink(const uint8_t *wire, uint16_t len) override;
    bool read_byte(uint8_t *out) override;
    bool is_open() const override;
    void on_tick(uint64_t now_ms) override;

    /** 设置 NAT 保活心跳（默认关闭）；hz<=0 表示禁用。 */
    void set_heartbeat(int hz, HeartbeatBuilder builder);

    /** 切换 GCS 目标地址（例如从 DNS 解析后写回，或运维侧切换中继）。 */
    void set_target(const std::string &ip, int port);

    Stats snapshot_stats() const;

private:
    int sendto_(const uint8_t *wire, uint16_t len);

    std::string bind_ip_;
    int         bind_port_ = 0;
    std::string target_ip_;
    int         target_port_ = 0;

    int fd_ = -1;
    std::deque<uint8_t> rx_buf_;
    std::vector<uint8_t> recv_tmp_;

    int               hb_hz_ = 0;
    uint64_t          hb_interval_ms_ = 0;
    uint64_t          last_hb_ms_ = 0;
    HeartbeatBuilder  hb_builder_;

    mutable std::mutex stat_mtx_;
    Stats              stats_{};
};

#endif
