#ifndef SWARM_HEALTH_MONITOR_H
#define SWARM_HEALTH_MONITOR_H

#include "comm/gcs_ip_link.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

/** 5G/IP 主链路健康监视。
 *  伴飞每个 tick 调用 sample(now_ms)，通过 GcsIpLink 提供的统计数据评估三态：
 *   - HEALTHY  ：最近 last_recv_ms 距今 ≤ recv_warn_ms
 *   - DEGRADED ：last_recv_ms 距今 ∈ (recv_warn_ms, recv_lost_ms]，或 send_errors 高发
 *   - LOST     ：last_recv_ms 距今 > recv_lost_ms，或 last_send_err_ms 持续超阈
 *  状态翻转时触发对应回调（HEALTHY→DEGRADED→LOST 单向通知，恢复时通知 on_recovered）。
 */
class HealthMonitor {
public:
    enum class State : uint8_t {
        HEALTHY  = 0,
        DEGRADED = 1,
        LOST     = 2,
    };

    struct Config {
        uint64_t sample_interval_ms = 500;
        uint64_t recv_warn_ms       = 3000;   ///< 主链 3s 无回包 → DEGRADED
        uint64_t recv_lost_ms       = 8000;   ///< 主链 8s 无回包 → LOST
        uint64_t recovery_hyst_ms   = 2000;   ///< 连续 2s 健康才恢复 HEALTHY
        uint64_t send_err_burst     = 5;      ///< 连续发送失败次数（窗口内）
        uint64_t send_err_window_ms = 2000;   ///< 发送失败统计窗口
        uint32_t status_repeat_ms   = 5000;   ///< STATUSTEXT 重复上报最小间隔
    };

    using StateCallback = std::function<void(State new_state,
                                             State old_state,
                                             const std::string &reason)>;
    using StatusEmitter = std::function<void(const std::string &text,
                                             uint8_t severity)>;

    HealthMonitor() = default;

    void set_link(GcsIpLink *link);
    void set_config(const Config &cfg);
    /** 状态变化回调（用于触发 LOITER/RTL 或 P900 fallback 等编排）。 */
    void set_state_callback(StateCallback cb);
    /** STATUSTEXT 发送回调（用于经 SwarmController 走令牌桶发回 GCS）。 */
    void set_status_emitter(StatusEmitter cb);
    /** 启动时机：GCS 链路已 open 后调用。 */
    void start(uint64_t now_ms);

    /** 在 receive_loop / control_loop 内高频调用，内部自带采样节流。 */
    void tick(uint64_t now_ms);

    State current_state() const { return state_.load(std::memory_order_relaxed); }
    bool  is_degraded()    const { return current_state() != State::HEALTHY; }
    bool  is_lost()        const { return current_state() == State::LOST; }

    /** 仅用于诊断：最近一次采样结果。 */
    struct Snapshot {
        State    state = State::HEALTHY;
        uint64_t since_last_recv_ms = 0;
        uint64_t tx_packets = 0;
        uint64_t rx_packets = 0;
        uint64_t send_errors = 0;
        uint64_t last_state_change_ms = 0;
    };
    Snapshot snapshot() const;

private:
    void transition_(State to, const std::string &reason, uint64_t now_ms);
    void maybe_emit_status_(const std::string &text,
                            uint8_t severity,
                            uint64_t now_ms);

    GcsIpLink     *link_ = nullptr;
    Config         cfg_{};
    StateCallback  state_cb_;
    StatusEmitter  status_cb_;

    std::atomic<State> state_{State::HEALTHY};
    uint64_t           last_sample_ms_  = 0;
    uint64_t           last_change_ms_  = 0;
    uint64_t           last_status_ms_  = 0;
    uint64_t           last_recv_seen_  = 0;
    uint64_t           healthy_since_   = 0;
    uint64_t           prev_send_errors_ = 0;
    uint64_t           send_err_window_start_ = 0;
    uint64_t           send_err_in_window_    = 0;

    mutable std::mutex stat_mtx_;
    Snapshot           snapshot_{};
};

#endif
