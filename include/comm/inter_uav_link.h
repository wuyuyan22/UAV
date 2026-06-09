#ifndef SWARM_INTER_UAV_LINK_H
#define SWARM_INTER_UAV_LINK_H

#include "hal/udp_socket.h"
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
/* 单入口包含 ardupilotmega 方言（内含 common 消息） */
#include <ardupilotmega/mavlink.h>

enum class LinkMsgType : uint8_t {
    STATE_REPORT  = 0x01,  // follower -> leader 状态上报
    FOLLOWER_CMD  = 0x06,  // leader -> follower 目标点
    CTRL_CMD      = 0x10,  // leader -> follower 控制指令
    TASK_META     = 0x20,  // leader -> follower 子任务元信息
    TASK_ITEM     = 0x21,  // leader -> follower 子任务航点项
    TASK_COMMIT   = 0x22,  // leader -> follower 子任务提交
    TASK_ACK      = 0x23,  // follower -> leader 子任务接收结果
    TASK_START    = 0x24,  // leader -> follower 子任务开始(AUTO)
};

// 机间通信的“内部数据结构”。
// 传输层已切换为 MAVLink 标准消息（不再使用自定义帧/Data96）。
struct LinkPacket {
    // 头部
    uint8_t     src_id   = 0;
    uint8_t     dst_id   = 0;
    LinkMsgType msg_type = LinkMsgType::STATE_REPORT;
    uint32_t    seq      = 0;

    // 载荷
    double  lat = 0;        // rad
    double  lon = 0;        // rad
    float   alt = 0;        // m
    float   vel = 0;        // m/s
    float   hdg = 0;        // rad
    float   pitch = 0;
    float   roll  = 0;
    uint16_t health = 0xFFFF;
    uint8_t armed = 0;
    uint8_t mode_valid = 0;
    uint32_t custom_mode = 0;

    /** CTRL_CMD：seq 为 CtrlProxyOp；ctrl_pf[0..5] 对应 MAVLink COMMAND_LONG param2..param7 */
    float   ctrl_pf[6] = {0, 0, 0, 0, 0, 0};
};

/** 机间/空中数据链路抽象：UDP 实现见 InterUavLink；P900 多从成帧实现可并列增加。 */
class ISwarmAirLink {
public:
    virtual ~ISwarmAirLink() = default;

    virtual bool bind(const std::string &ip, int port) = 0;
    virtual void set_target(const std::string &ip, int port) = 0;
    virtual void set_local_identity(uint8_t sysid,
                                   uint8_t compid = MAV_COMP_ID_ONBOARD_COMPUTER) = 0;
    virtual int  send_packet(const LinkPacket &pkt) = 0;
    virtual bool recv_packet(LinkPacket &pkt, int timeout_ms = 5) = 0;
};

class InterUavLink : public ISwarmAirLink {
public:
    InterUavLink() = default;
    ~InterUavLink() override = default;

    bool bind(const std::string &ip, int port) override;
    void set_target(const std::string &ip, int port) override;
    void set_local_identity(uint8_t sysid, uint8_t compid = MAV_COMP_ID_ONBOARD_COMPUTER) override;

    int send_packet(const LinkPacket &pkt) override;
    bool recv_packet(LinkPacket &pkt, int timeout_ms = 5) override;

private:
    struct PeerEndpoint {
        std::string ip;
        int         port = 0;
    };

    void note_peer(uint8_t sysid, const std::string &ip, int port);

    UdpSocket sock_;
    std::string target_ip_;
    int         target_port_ = 0;
    uint8_t     local_sysid_ = 1;
    uint8_t     local_compid_ = MAV_COMP_ID_ONBOARD_COMPUTER;
    mavlink_status_t mav_status_{};

    /** 长机：僚机 STATE_REPORT 等报文的 UDP 源地址（sysid -> ip:port） */
    std::unordered_map<uint8_t, PeerEndpoint> peers_;
    std::mutex peer_mtx_;
};

#endif
