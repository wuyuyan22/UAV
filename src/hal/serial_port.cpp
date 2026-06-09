#include "hal/serial_port.h"

#ifdef __linux__
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <cstdio>
#include <cerrno>
#include <cstring>

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string &device, int baudrate) {
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        fprintf(stderr, "[serial] open %s failed: %s\n", device.c_str(), strerror(errno));
        return false;
    }
    if (configure(baudrate) < 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    fprintf(stdout, "[serial] %s opened @ %d baud\n", device.c_str(), baudrate);
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

int SerialPort::read_bytes(uint8_t *buf, size_t max_len) {
    if (fd_ < 0) return -1;
    int n = ::read(fd_, buf, max_len);
    if (n < 0 && errno == EAGAIN) return 0;
    return n;
}

int SerialPort::write_bytes(const uint8_t *buf, size_t len) {
    if (fd_ < 0) return -1;
    return ::write(fd_, buf, len);
}

int SerialPort::configure(int baudrate) {
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd_, &tty) != 0) return -1;

    speed_t baud;
    switch (baudrate) {
    case 9600:   baud = B9600;   break;
    case 19200:  baud = B19200;  break;
    case 38400:  baud = B38400;  break;
    case 57600:  baud = B57600;  break;
    case 115200: baud = B115200; break;
    case 230400: baud = B230400; break;
    case 460800: baud = B460800; break;
    case 921600: baud = B921600; break;
    default:     baud = B115200; break;
    }

    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK |
                      ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1; // 100ms timeout

    return tcsetattr(fd_, TCSANOW, &tty);
}

#else
// Windows 桩实现 -- 实际部署在鲁班猫 Linux 上
SerialPort::~SerialPort() {}
bool SerialPort::open(const std::string &, int) { return false; }
void SerialPort::close() {}
int  SerialPort::read_bytes(uint8_t *, size_t) { return -1; }
int  SerialPort::write_bytes(const uint8_t *, size_t) { return -1; }
int  SerialPort::configure(int) { return -1; }
#endif
