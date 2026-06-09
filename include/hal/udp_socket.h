#ifndef SWARM_UDP_SOCKET_H
#define SWARM_UDP_SOCKET_H

#include <cstdint>
#include <cstddef>
#include <netinet/in.h>
#include <string>

class UdpSocket {
public:
    UdpSocket() = default;
    ~UdpSocket();

    bool bind_local(const std::string &ip, int port);
    bool set_target(const std::string &ip, int port);
    void close();
    bool is_open() const { return fd_ >= 0; }

    int recv_bytes(uint8_t *buf, size_t max_len, int timeout_ms = 100);

    /** 收 UDP 并返回对端地址（用于长机学习僚机源 IP） */
    int recv_from(uint8_t *buf, size_t max_len, int timeout_ms,
                  std::string *out_ip, int *out_port);

    int send_bytes(const uint8_t *buf, size_t len);

    int send_to(const uint8_t *buf, size_t len,
                const std::string &ip, int port);

private:
    int fd_ = -1;
    struct sockaddr_in target_addr_;
    bool target_set_ = false;
};

#endif
