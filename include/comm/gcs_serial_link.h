#ifndef SWARM_GCS_SERIAL_LINK_H
#define SWARM_GCS_SERIAL_LINK_H

#include "comm/gcs_link.h"
#include "hal/serial_port.h"

#include <string>

/** 旧串口 GCS 直连实现（兜底/调试用）。
 *  外部传入一个 SerialPort& 引用，不接管所有权。 */
class GcsSerialLink : public IGcsLink {
public:
    GcsSerialLink(SerialPort &port, std::string device, int baudrate);
    ~GcsSerialLink() override = default;

    bool open() override;
    void close() override;
    int  write_mavlink(const uint8_t *wire, uint16_t len) override;
    bool read_byte(uint8_t *out) override;
    bool is_open() const override;

private:
    SerialPort &port_;
    std::string device_;
    int         baud_ = 0;
};

#endif
