#include "hal/udp_socket.h"

#ifdef __linux__
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <cstring>
#include <cstdio>

UdpSocket::~UdpSocket() { close(); }

bool UdpSocket::bind_local(const std::string &ip, int port) {
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) return false;

    int reuse = 1;
    setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(port);
    addr.sin_addr.s_addr = inet_addr(ip.c_str());

    if (::bind(fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "[udp] bind %s:%d failed\n", ip.c_str(), port);
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    fcntl(fd_, F_SETFL, fcntl(fd_, F_GETFL) | O_NONBLOCK);
    fprintf(stdout, "[udp] bound %s:%d\n", ip.c_str(), port);
    return true;
}

bool UdpSocket::set_target(const std::string &ip, int port) {
    memset(&target_addr_, 0, sizeof(target_addr_));
    target_addr_.sin_family      = AF_INET;
    target_addr_.sin_port        = htons(port);
    target_addr_.sin_addr.s_addr = inet_addr(ip.c_str());
    target_set_ = true;

    if (fd_ < 0) {
        fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) return false;
        fcntl(fd_, F_SETFL, fcntl(fd_, F_GETFL) | O_NONBLOCK);
    }
    return true;
}

void UdpSocket::close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

int UdpSocket::recv_from(uint8_t *buf, size_t max_len, int timeout_ms,
                         std::string *out_ip, int *out_port) {
    if (fd_ < 0) return -1;
    struct pollfd pfd = { fd_, POLLIN, 0 };
    int ret = poll(&pfd, 1, timeout_ms);
    if (ret <= 0) return 0;

    struct sockaddr_in from;
    socklen_t fromlen = sizeof(from);
    int n = recvfrom(fd_, buf, max_len, 0, (struct sockaddr *)&from, &fromlen);
    if (n < 0) return 0;

    if (out_ip) {
        char ipbuf[INET_ADDRSTRLEN];
        if (inet_ntop(AF_INET, &from.sin_addr, ipbuf, sizeof(ipbuf)))
            *out_ip = ipbuf;
        else
            out_ip->clear();
    }
    if (out_port)
        *out_port = ntohs(from.sin_port);
    return n;
}

int UdpSocket::recv_bytes(uint8_t *buf, size_t max_len, int timeout_ms) {
    return recv_from(buf, max_len, timeout_ms, nullptr, nullptr);
}

int UdpSocket::send_bytes(const uint8_t *buf, size_t len) {
    if (fd_ < 0 || !target_set_) return -1;
    return sendto(fd_, buf, len, 0,
                  (struct sockaddr *)&target_addr_, sizeof(target_addr_));
}

int UdpSocket::send_to(const uint8_t *buf, size_t len,
                        const std::string &ip, int port) {
    if (fd_ < 0) return -1;
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(port);
    addr.sin_addr.s_addr = inet_addr(ip.c_str());
    return sendto(fd_, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
}

#else
UdpSocket::~UdpSocket() {}
bool UdpSocket::bind_local(const std::string &, int) { return false; }
bool UdpSocket::set_target(const std::string &, int) { return false; }
void UdpSocket::close() {}
int  UdpSocket::recv_bytes(uint8_t *, size_t, int) { return -1; }
int  UdpSocket::recv_from(uint8_t *, size_t, int, std::string *, int *) { return -1; }
int  UdpSocket::send_bytes(const uint8_t *, size_t) { return -1; }
int  UdpSocket::send_to(const uint8_t *, size_t, const std::string &, int) { return -1; }
#endif
