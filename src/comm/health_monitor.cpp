#include "comm/health_monitor.h"

#include <cstdio>
#include <utility>

// MAVLink STATUSTEXT severity 取值与 common.xml 一致（仅头文件未引入此处，避免循环依赖）
namespace {
constexpr uint8_t SEV_NOTICE   = 5;
constexpr uint8_t SEV_WARNING  = 4;
constexpr uint8_t SEV_CRITICAL = 2;
}  // namespace

void HealthMonitor::set_link(GcsIpLink *link) { link_ = link; }

void HealthMonitor::set_config(const Config &cfg) { cfg_ = cfg; }

void HealthMonitor::set_state_callback(StateCallback cb) {
    state_cb_ = std::move(cb);
}

void HealthMonitor::set_status_emitter(StatusEmitter cb) {
    status_cb_ = std::move(cb);
}

void HealthMonitor::start(uint64_t now_ms) {
    state_.store(State::HEALTHY, std::memory_order_relaxed);
    last_sample_ms_ = 0;
    last_change_ms_ = now_ms;
    last_status_ms_ = 0;
    last_recv_seen_ = 0;
    healthy_since_  = now_ms;
    prev_send_errors_      = 0;
    send_err_window_start_ = now_ms;
    send_err_in_window_    = 0;
    std::lock_guard<std::mutex> lk(stat_mtx_);
    snapshot_ = Snapshot{};
    snapshot_.state = State::HEALTHY;
}

void HealthMonitor::tick(uint64_t now_ms) {
    if (link_ == nullptr) return;
    if (cfg_.sample_interval_ms == 0) cfg_.sample_interval_ms = 500;
    if (last_sample_ms_ != 0 && (now_ms - last_sample_ms_) < cfg_.sample_interval_ms)
        return;
    last_sample_ms_ = now_ms;

    const GcsIpLink::Stats st = link_->snapshot_stats();

    // 发送错误突发统计（滑窗近似）
    if (st.send_errors > prev_send_errors_) {
        send_err_in_window_ += (st.send_errors - prev_send_errors_);
    }
    prev_send_errors_ = st.send_errors;
    if (now_ms - send_err_window_start_ > cfg_.send_err_window_ms) {
        send_err_window_start_ = now_ms;
        send_err_in_window_    = 0;
    }

    // since_last_recv：从未收到时按 since start 计
    const uint64_t base_ms = (st.last_recv_ms != 0) ? st.last_recv_ms : last_change_ms_;
    const uint64_t since_recv = (now_ms > base_ms) ? (now_ms - base_ms) : 0;

    // 评估期望状态
    State want = State::HEALTHY;
    std::string reason;
    if (since_recv > cfg_.recv_lost_ms) {
        want = State::LOST;
        reason = "no GCS recv > " + std::to_string(cfg_.recv_lost_ms) + "ms";
    } else if (since_recv > cfg_.recv_warn_ms) {
        want = State::DEGRADED;
        reason = "no GCS recv > " + std::to_string(cfg_.recv_warn_ms) + "ms";
    } else if (send_err_in_window_ >= cfg_.send_err_burst) {
        want = State::DEGRADED;
        reason = "send errors " + std::to_string(send_err_in_window_) +
                 " in " + std::to_string(cfg_.send_err_window_ms) + "ms";
    }

    const State cur = state_.load(std::memory_order_relaxed);

    // 恢复需要迟滞：连续 recovery_hyst_ms 都期望 HEALTHY 才回切
    if (want == State::HEALTHY && cur != State::HEALTHY) {
        if (healthy_since_ == 0) healthy_since_ = now_ms;
        if ((now_ms - healthy_since_) >= cfg_.recovery_hyst_ms) {
            transition_(State::HEALTHY, "recovered", now_ms);
        }
    } else if (want != State::HEALTHY) {
        healthy_since_ = 0;
        if (want != cur) {
            transition_(want, reason, now_ms);
        } else {
            // 同态持续，周期性重报告
            uint8_t sev = (want == State::LOST) ? SEV_CRITICAL : SEV_WARNING;
            maybe_emit_status_("[5G] " + reason, sev, now_ms);
        }
    } else {
        healthy_since_ = now_ms;
    }

    {
        std::lock_guard<std::mutex> lk(stat_mtx_);
        snapshot_.state               = state_.load(std::memory_order_relaxed);
        snapshot_.since_last_recv_ms  = since_recv;
        snapshot_.tx_packets          = st.tx_packets;
        snapshot_.rx_packets          = st.rx_packets;
        snapshot_.send_errors         = st.send_errors;
        snapshot_.last_state_change_ms = last_change_ms_;
    }
}

void HealthMonitor::transition_(State to, const std::string &reason, uint64_t now_ms) {
    const State from = state_.exchange(to, std::memory_order_relaxed);
    if (from == to) return;
    last_change_ms_ = now_ms;
    last_status_ms_ = 0;  // 强制重新发一条 STATUSTEXT

    const char *from_s = (from == State::HEALTHY) ? "HEALTHY"
                       : (from == State::DEGRADED) ? "DEGRADED" : "LOST";
    const char *to_s   = (to == State::HEALTHY)   ? "HEALTHY"
                       : (to == State::DEGRADED)   ? "DEGRADED" : "LOST";
    std::fprintf(stderr, "[5GHealth] %s -> %s : %s\n", from_s, to_s, reason.c_str());

    uint8_t sev = (to == State::LOST) ? SEV_CRITICAL
                 : (to == State::DEGRADED) ? SEV_WARNING : SEV_NOTICE;
    std::string text;
    text.reserve(64);
    text.append("[5G] ").append(to_s);
    if (!reason.empty()) text.append(": ").append(reason);
    maybe_emit_status_(text, sev, now_ms);

    if (state_cb_) state_cb_(to, from, reason);
}

void HealthMonitor::maybe_emit_status_(const std::string &text,
                                       uint8_t severity,
                                       uint64_t now_ms) {
    if (!status_cb_) return;
    if (last_status_ms_ != 0 && (now_ms - last_status_ms_) < cfg_.status_repeat_ms)
        return;
    last_status_ms_ = now_ms;
    status_cb_(text, severity);
}

HealthMonitor::Snapshot HealthMonitor::snapshot() const {
    std::lock_guard<std::mutex> lk(stat_mtx_);
    return snapshot_;
}
