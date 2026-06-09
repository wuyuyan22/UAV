#include "comm/gcs_ip_link.h"

#include <algorithm>
#include <chrono>
#include <utility>

#ifdef __linux__
#include <arpa/inet.h>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {

uint64_t now_ms_steady() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
}

}  // namespace

GcsIpLink::GcsIpLink(const std::string &bind_ip,
                     int bind_port,
                     const std::string &target_ip,
                     int target_port)
    : bind_ip_(bind_ip),
      bind_port_(bind_port),
      target_ip_(target_ip),
      target_port_(target_port),
      recv_tmp_(1500, 0) {}

GcsIpLink::~GcsIpLink() { close(); }

bool GcsIpLink::open() {
    close();
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        std::fprintf(stderr, "[gcs_ip] socket create failed: %s\n", std::strerror(errno));
        return false;
    }

    int reuse = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    ::fcntl(fd_, F_SETFL, ::fcntl(fd_, F_GETFL) | O_NONBLOCK);

    // DSCP AF41 (0x88)：向蜂窝/中转网络暗示 GCS 流量为高优先级遥测控制。
    int tos = 0x88;
    ::setsockopt(fd_, IPPROTO_IP, IP_TOS, &tos, sizeof(tos));

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(static_cast<uint16_t>(bind_port_));
    local.sin_addr.s_addr = bind_ip_.empty() ? INADDR_ANY
                                             : ::inet_addr(bind_ip_.c_str());
    if (::bind(fd_, reinterpret_cast<sockaddr *>(&local), sizeof(local)) < 0) {
        std::fprintf(stderr, "[gcs_ip] bind %s:%d failed: %s\n",
                     bind_ip_.c_str(), bind_port_, std::strerror(errno));
        close();
        return false;
    }

    {
        std::lock_guard<std::mutex> lk(stat_mtx_);
        stats_ = Stats{};
    }
    last_hb_ms_ = 0;

    std::fprintf(stdout, "[gcs_ip] bind %s:%d -> target %s:%d  (hb=%dHz)\n",
                 bind_ip_.c_str(), bind_port_, target_ip_.c_str(), target_port_, hb_hz_);
    return true;
}

void GcsIpLink::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    rx_buf_.clear();
}

int GcsIpLink::sendto_(const uint8_t *wire, uint16_t len) {
    if (fd_ < 0 || wire == nullptr || len == 0) return -1;
    sockaddr_in peer{};
    peer.sin_family = AF_INET;
    peer.sin_port = htons(static_cast<uint16_t>(target_port_));
    peer.sin_addr.s_addr = ::inet_addr(target_ip_.c_str());
    const int n = ::sendto(fd_, wire, len, 0,
                           reinterpret_cast<sockaddr *>(&peer), sizeof(peer));

    std::lock_guard<std::mutex> lk(stat_mtx_);
    if (n > 0) {
        stats_.tx_packets++;
        stats_.tx_bytes += static_cast<uint64_t>(n);
        stats_.last_send_ms = now_ms_steady();
    } else {
        stats_.send_errors++;
        stats_.last_send_err_ms = now_ms_steady();
        stats_.last_send_err_no = static_cast<uint64_t>(errno);
    }
    return n;
}

int GcsIpLink::write_mavlink(const uint8_t *wire, uint16_t len) {
    return sendto_(wire, len);
}

bool GcsIpLink::read_byte(uint8_t *out) {
    if (out == nullptr || fd_ < 0) return false;
    if (rx_buf_.empty()) {
        pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = POLLIN;
        const int ready = ::poll(&pfd, 1, 0);
        if (ready <= 0) return false;

        sockaddr_in from{};
        socklen_t from_len = sizeof(from);
        const int n = ::recvfrom(fd_, recv_tmp_.data(), recv_tmp_.size(), 0,
                                 reinterpret_cast<sockaddr *>(&from), &from_len);
        if (n <= 0) return false;
        for (int i = 0; i < n; ++i) rx_buf_.push_back(recv_tmp_[i]);
        std::lock_guard<std::mutex> lk(stat_mtx_);
        stats_.rx_packets++;
        stats_.rx_bytes += static_cast<uint64_t>(n);
        stats_.last_recv_ms = now_ms_steady();
    }

    if (rx_buf_.empty()) return false;
    *out = rx_buf_.front();
    rx_buf_.pop_front();
    return true;
}

bool GcsIpLink::is_open() const { return fd_ >= 0; }

void GcsIpLink::on_tick(uint64_t now_ms) {
    if (fd_ < 0) return;
    if (hb_hz_ <= 0 || !hb_builder_) return;
    if (hb_interval_ms_ == 0) return;
    if (last_hb_ms_ != 0 && (now_ms - last_hb_ms_) < hb_interval_ms_) return;

    uint8_t buf[280];  // MAVLink2 最大约 280B
    const uint16_t len = hb_builder_(buf, sizeof(buf));
    if (len == 0) return;

    const int n = sendto_(buf, len);
    last_hb_ms_ = now_ms;
    if (n > 0) {
        std::lock_guard<std::mutex> lk(stat_mtx_);
        stats_.last_heartbeat_tx_ms = now_ms;
    }
}

void GcsIpLink::set_heartbeat(int hz, HeartbeatBuilder builder) {
    hb_hz_ = hz;
    hb_builder_ = std::move(builder);
    hb_interval_ms_ = (hz > 0) ? static_cast<uint64_t>(1000 / std::max(1, hz)) : 0;
    last_hb_ms_ = 0;
}

void GcsIpLink::set_target(const std::string &ip, int port) {
    if (ip.empty() || port <= 0) return;
    target_ip_ = ip;
    target_port_ = port;
    std::fprintf(stdout, "[gcs_ip] target updated -> %s:%d\n",
                 target_ip_.c_str(), target_port_);
}

GcsIpLink::Stats GcsIpLink::snapshot_stats() const {
    std::lock_guard<std::mutex> lk(stat_mtx_);
    return stats_;
}

#else  // !__linux__

GcsIpLink::GcsIpLink(const std::string &, int, const std::string &, int) {}
GcsIpLink::~GcsIpLink() {}
bool GcsIpLink::open() { return false; }
void GcsIpLink::close() {}
int  GcsIpLink::write_mavlink(const uint8_t *, uint16_t) { return -1; }
bool GcsIpLink::read_byte(uint8_t *) { return false; }
bool GcsIpLink::is_open() const { return false; }
void GcsIpLink::on_tick(uint64_t) {}
void GcsIpLink::set_heartbeat(int, HeartbeatBuilder) {}
void GcsIpLink::set_target(const std::string &, int) {}
int  GcsIpLink::sendto_(const uint8_t *, uint16_t) { return -1; }
GcsIpLink::Stats GcsIpLink::snapshot_stats() const { return Stats{}; }

#endif
