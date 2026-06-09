#ifndef SWARM_SERIAL_PORT_H
#define SWARM_SERIAL_PORT_H

#include <cstdint>
#include <cstddef>
#include <string>

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    bool open(const std::string &device, int baudrate);
    void close();
    bool is_open() const { return fd_ >= 0; }

    int read_bytes(uint8_t *buf, size_t max_len);
    int write_bytes(const uint8_t *buf, size_t len);

private:
    int fd_ = -1;
    int configure(int baudrate);
};

#endif
