#pragma once

#include <cstdint>
#include <cstddef>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace canopen {

constexpr uint16_t NMT_ID         = 0x000;
constexpr uint16_t TPDO1_BASE     = 0x180;
constexpr uint16_t RPDO1_BASE     = 0x200;
constexpr uint16_t TPDO2_BASE     = 0x280;
constexpr uint16_t SDO_TX_BASE    = 0x580;
constexpr uint16_t SDO_RX_BASE    = 0x600;
constexpr uint16_t HEARTBEAT_BASE = 0x700;

constexpr uint8_t NMT_OPERATIONAL     = 0x01;
constexpr uint8_t NMT_STOPPED         = 0x02;
constexpr uint8_t NMT_PRE_OPERATIONAL = 0x80;
constexpr uint8_t NMT_RESET_NODE      = 0x81;
constexpr uint8_t NMT_RESET_COMM      = 0x82;

constexpr uint8_t STATE_OPERATIONAL     = 0x05;
constexpr uint8_t STATE_PRE_OPERATIONAL = 0x7F;

} // namespace canopen

class CANSocket {
public:
    CANSocket() = default;
    ~CANSocket();

    CANSocket(const CANSocket&) = delete;
    CANSocket& operator=(const CANSocket&) = delete;

    bool open(const char* interface_name);
    void close();
    bool is_open() const { return fd_ >= 0; }

    bool send_can(uint32_t can_id, const void* data, uint8_t len);
    bool send_canfd(uint32_t can_id, const void* data, uint8_t len, bool brs = true);

    // Returns true if a frame was received; false on timeout or error.
    bool recv(struct canfd_frame& frame, int timeout_ms);

private:
    int fd_ = -1;
};

enum class SDOStatus { OK, Timeout, Aborted, Error };
const char* sdo_status_str(SDOStatus s);

SDOStatus sdo_read(CANSocket& sock, uint8_t node_id, uint16_t index, uint8_t sub,
                   void* out, size_t out_size, int timeout_ms = 1000);

SDOStatus sdo_write(CANSocket& sock, uint8_t node_id, uint16_t index, uint8_t sub,
                    const void* data, size_t size, int timeout_ms = 1000);

SDOStatus sdo_read_u32(CANSocket& s, uint8_t nid, uint16_t idx, uint8_t sub, uint32_t& v);
SDOStatus sdo_read_f32(CANSocket& s, uint8_t nid, uint16_t idx, uint8_t sub, float& v);
SDOStatus sdo_write_u8(CANSocket& s, uint8_t nid, uint16_t idx, uint8_t sub, uint8_t v);
SDOStatus sdo_write_i8(CANSocket& s, uint8_t nid, uint16_t idx, uint8_t sub, int8_t v);
SDOStatus sdo_write_u16(CANSocket& s, uint8_t nid, uint16_t idx, uint8_t sub, uint16_t v);
SDOStatus sdo_write_u32(CANSocket& s, uint8_t nid, uint16_t idx, uint8_t sub, uint32_t v);

bool nmt_send(CANSocket& sock, uint8_t command, uint8_t node_id);
bool heartbeat_send(CANSocket& sock, uint8_t node_id, uint8_t state);
