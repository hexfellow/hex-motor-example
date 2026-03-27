#include "canopen.hpp"

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>

// ---------------------------------------------------------------------------
// CANSocket
// ---------------------------------------------------------------------------

CANSocket::~CANSocket() { close(); }

bool CANSocket::open(const char* interface_name) {
    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) {
        std::fprintf(stderr, "socket(PF_CAN): %s\n", std::strerror(errno));
        return false;
    }

    int enable_fd = 1;
    if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd)) < 0) {
        std::fprintf(stderr, "setsockopt CAN_RAW_FD_FRAMES: %s\n", std::strerror(errno));
        close();
        return false;
    }

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, interface_name, IFNAMSIZ - 1);
    if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
        std::fprintf(stderr, "ioctl SIOCGIFINDEX(%s): %s\n", interface_name, std::strerror(errno));
        close();
        return false;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::fprintf(stderr, "bind(%s): %s\n", interface_name, std::strerror(errno));
        close();
        return false;
    }

    return true;
}

void CANSocket::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool CANSocket::send_can(uint32_t can_id, const void* data, uint8_t len) {
    if (len > 8) return false;
    struct can_frame frame{};
    frame.can_id = can_id;
    frame.can_dlc = len;
    if (data && len > 0) std::memcpy(frame.data, data, len);
    return ::write(fd_, &frame, sizeof(frame)) == sizeof(frame);
}

bool CANSocket::send_canfd(uint32_t can_id, const void* data, uint8_t len, bool brs) {
    if (len > 64) return false;
    struct canfd_frame frame{};
    frame.can_id = can_id;
    frame.len = len;
#ifdef CANFD_FDF
    frame.flags = CANFD_FDF | (brs ? CANFD_BRS : 0);
#else
    frame.flags = brs ? CANFD_BRS : 0;
#endif
    if (data && len > 0) std::memcpy(frame.data, data, len);
    return ::write(fd_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
}

bool CANSocket::recv(struct canfd_frame& frame, int timeout_ms) {
    struct pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;
    int ret = ::poll(&pfd, 1, timeout_ms);
    if (ret <= 0) return false;

    ssize_t nbytes = ::read(fd_, &frame, sizeof(frame));
    return nbytes > 0;
}

// ---------------------------------------------------------------------------
// SDO helpers
// ---------------------------------------------------------------------------

const char* sdo_status_str(SDOStatus s) {
    switch (s) {
        case SDOStatus::OK:      return "OK";
        case SDOStatus::Timeout: return "Timeout";
        case SDOStatus::Aborted: return "Aborted";
        case SDOStatus::Error:   return "Error";
    }
    return "Unknown";
}

static bool wait_sdo_response(CANSocket& sock, uint8_t node_id,
                               uint8_t resp[8], int timeout_ms) {
    uint32_t expected_id = canopen::SDO_TX_BASE + node_id;
    struct canfd_frame frame{};

    while (timeout_ms > 0) {
        auto t0 = std::chrono::steady_clock::now();
        if (!sock.recv(frame, timeout_ms)) return false;
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0).count();
        timeout_ms -= static_cast<int>(elapsed);
        if (timeout_ms < 0) timeout_ms = 0;

        if ((frame.can_id & CAN_SFF_MASK) == expected_id && frame.len >= 8) {
            std::memcpy(resp, frame.data, 8);
            return true;
        }
    }
    return false;
}

SDOStatus sdo_read(CANSocket& sock, uint8_t node_id, uint16_t index, uint8_t sub,
                   void* out, size_t out_size, int timeout_ms) {
    uint8_t req[8]{};
    req[0] = 0x40;
    req[1] = index & 0xFF;
    req[2] = (index >> 8) & 0xFF;
    req[3] = sub;

    if (!sock.send_can(canopen::SDO_RX_BASE + node_id, req, 8))
        return SDOStatus::Error;

    uint8_t resp[8]{};
    if (!wait_sdo_response(sock, node_id, resp, timeout_ms))
        return SDOStatus::Timeout;

    if (resp[0] == 0x80)
        return SDOStatus::Aborted;

    uint8_t cmd = resp[0];
    size_t data_len = 4;
    if (cmd & 0x01) { // size indicated
        size_t n = (cmd >> 2) & 0x03;
        data_len = 4 - n;
    }

    size_t copy_len = data_len < out_size ? data_len : out_size;
    std::memcpy(out, resp + 4, copy_len);
    return SDOStatus::OK;
}

SDOStatus sdo_write(CANSocket& sock, uint8_t node_id, uint16_t index, uint8_t sub,
                    const void* data, size_t size, int timeout_ms) {
    if (size == 0 || size > 4) return SDOStatus::Error;

    uint8_t n = static_cast<uint8_t>(4 - size);
    uint8_t cmd = static_cast<uint8_t>(0x20 | (n << 2) | 0x03); // expedited + size indicated

    uint8_t req[8]{};
    req[0] = cmd;
    req[1] = index & 0xFF;
    req[2] = (index >> 8) & 0xFF;
    req[3] = sub;
    std::memcpy(req + 4, data, size);

    if (!sock.send_can(canopen::SDO_RX_BASE + node_id, req, 8))
        return SDOStatus::Error;

    uint8_t resp[8]{};
    if (!wait_sdo_response(sock, node_id, resp, timeout_ms))
        return SDOStatus::Timeout;

    if (resp[0] == 0x80)
        return SDOStatus::Aborted;

    if (resp[0] != 0x60)
        return SDOStatus::Error;

    return SDOStatus::OK;
}

// Convenience wrappers

SDOStatus sdo_read_u32(CANSocket& s, uint8_t nid, uint16_t idx, uint8_t sub, uint32_t& v) {
    v = 0;
    return sdo_read(s, nid, idx, sub, &v, sizeof(v));
}

SDOStatus sdo_read_f32(CANSocket& s, uint8_t nid, uint16_t idx, uint8_t sub, float& v) {
    v = 0.0f;
    return sdo_read(s, nid, idx, sub, &v, sizeof(v));
}

SDOStatus sdo_write_u8(CANSocket& s, uint8_t nid, uint16_t idx, uint8_t sub, uint8_t v) {
    return sdo_write(s, nid, idx, sub, &v, sizeof(v));
}

SDOStatus sdo_write_i8(CANSocket& s, uint8_t nid, uint16_t idx, uint8_t sub, int8_t v) {
    return sdo_write(s, nid, idx, sub, &v, sizeof(v));
}

SDOStatus sdo_write_u16(CANSocket& s, uint8_t nid, uint16_t idx, uint8_t sub, uint16_t v) {
    return sdo_write(s, nid, idx, sub, &v, sizeof(v));
}

SDOStatus sdo_write_u32(CANSocket& s, uint8_t nid, uint16_t idx, uint8_t sub, uint32_t v) {
    return sdo_write(s, nid, idx, sub, &v, sizeof(v));
}

// ---------------------------------------------------------------------------
// NMT / Heartbeat
// ---------------------------------------------------------------------------

bool nmt_send(CANSocket& sock, uint8_t command, uint8_t node_id) {
    uint8_t data[2] = {command, node_id};
    return sock.send_can(canopen::NMT_ID, data, 2);
}

bool heartbeat_send(CANSocket& sock, uint8_t node_id, uint8_t state) {
    return sock.send_can(canopen::HEARTBEAT_BASE + node_id, &state, 1);
}
