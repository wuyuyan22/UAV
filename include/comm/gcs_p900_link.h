#ifndef SWARM_GCS_P900_LINK_H
#define SWARM_GCS_P900_LINK_H

#include "comm/gcs_link.h"
#include "comm/p900_framed_link.h"

/** P900 成帧通道上的 GCS 子流（仅 Leader 兜底使用）。
 *  P900 串口由外部 P900FramedLink 实例打开/管理；本类只做协议桥接。 */
class GcsP900Link : public IGcsLink {
public:
    explicit GcsP900Link(P900FramedLink &link);
    ~GcsP900Link() override = default;

    bool open() override;
    void close() override;
    int  write_mavlink(const uint8_t *wire, uint16_t len) override;
    bool read_byte(uint8_t *out) override;
    bool is_open() const override;

private:
    P900FramedLink &link_;
};

#endif
