#include "comm/gcs_p900_link.h"

GcsP900Link::GcsP900Link(P900FramedLink &link) : link_(link) {}

// 注意：底层 P900 串口由 SwarmController 在 init() 后续阶段统一打开/关闭。
// 因此本类的 open/close 仅返回真，避免在 GCS 链路构造期间因串口尚未就绪而失败。
bool GcsP900Link::open() { return true; }

void GcsP900Link::close() {}

int GcsP900Link::write_mavlink(const uint8_t *wire, uint16_t len) {
    if (!link_.is_open() || wire == nullptr || len == 0) return -1;
    return link_.write_mavlink_to_gcs(wire, len);
}

bool GcsP900Link::read_byte(uint8_t *out) {
    if (out == nullptr) return false;
    return link_.pop_gcs_byte(out);
}

bool GcsP900Link::is_open() const { return link_.is_open(); }
