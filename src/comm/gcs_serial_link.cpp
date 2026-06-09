#include "comm/gcs_serial_link.h"

#include <utility>

GcsSerialLink::GcsSerialLink(SerialPort &port, std::string device, int baudrate)
    : port_(port), device_(std::move(device)), baud_(baudrate) {}

bool GcsSerialLink::open() {
    if (port_.is_open()) return true;
    return port_.open(device_, baud_);
}

void GcsSerialLink::close() {
    port_.close();
}

int GcsSerialLink::write_mavlink(const uint8_t *wire, uint16_t len) {
    if (!port_.is_open() || wire == nullptr || len == 0) return -1;
    size_t off = 0;
    while (off < len) {
        const int n = port_.write_bytes(wire + off, len - off);
        if (n <= 0) return -1;
        off += static_cast<size_t>(n);
    }
    return static_cast<int>(off);
}

bool GcsSerialLink::read_byte(uint8_t *out) {
    if (out == nullptr || !port_.is_open()) return false;
    uint8_t b = 0;
    const int n = port_.read_bytes(&b, 1);
    if (n <= 0) return false;
    *out = b;
    return true;
}

bool GcsSerialLink::is_open() const { return port_.is_open(); }
