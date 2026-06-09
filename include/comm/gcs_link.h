#ifndef SWARM_GCS_LINK_H
#define SWARM_GCS_LINK_H

#include <cstdint>

class IGcsLink {
public:
    virtual ~IGcsLink() = default;
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual int write_mavlink(const uint8_t *wire, uint16_t len) = 0;
    virtual bool read_byte(uint8_t *out) = 0;
    virtual bool is_open() const = 0;
    virtual void on_tick(uint64_t now_ms) { (void)now_ms; }
};

#endif
