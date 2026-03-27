#include "hex_motor.hpp"

#include <cstdio>
#include <cstring>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// MitTarget encoding
// ---------------------------------------------------------------------------

static uint32_t float_to_uint(float x, float x_min, float x_max, uint32_t bits) {
    float span  = x_max - x_min;
    float scale = static_cast<float>((1u << bits) - 1);
    return static_cast<uint32_t>((x - x_min) * scale / span);
}

void MitTarget::encode(const MitTargetMapping& m, uint8_t out[8]) const {
    float p = std::clamp(position, m.position_min, m.position_max);
    float v = std::clamp(velocity, m.velocity_min, m.velocity_max);
    float t = std::clamp(torque,   m.torque_min,   m.torque_max);
    float k = std::clamp(kp,       m.kp_min,       m.kp_max);
    float d = std::clamp(kd,       m.kd_min,       m.kd_max);

    uint32_t pos_u = float_to_uint(p, m.position_min, m.position_max, 16);
    uint32_t vel_u = float_to_uint(v, m.velocity_min, m.velocity_max, 12);
    uint32_t trq_u = float_to_uint(t, m.torque_min,   m.torque_max,   12);
    uint32_t kp_u  = float_to_uint(k, m.kp_min,       m.kp_max,       12);
    uint32_t kd_u  = float_to_uint(d, m.kd_min,       m.kd_max,       12);

    uint32_t lo = trq_u | (kd_u << 12) | ((kp_u & 0xFF) << 24);
    uint32_t hi = (kp_u >> 8) | (vel_u << 4) | (pos_u << 16);

    std::memcpy(out,     &lo, 4);
    std::memcpy(out + 4, &hi, 4);
}

// ---------------------------------------------------------------------------
// HexMotor
// ---------------------------------------------------------------------------

HexMotor::HexMotor(uint8_t node_id, const MitTargetMapping& mapping)
    : node_id_(node_id), mapping_(mapping) {}

#define SDO_CHECK(expr, msg)                                        \
    do {                                                            \
        SDOStatus _s = (expr);                                      \
        if (_s != SDOStatus::OK) {                                  \
            std::fprintf(stderr, "Motor 0x%02X: %s (%s)\n",        \
                         node_id_, msg, sdo_status_str(_s));        \
            return false;                                           \
        }                                                           \
    } while (0)

bool HexMotor::init(CANSocket& sock) {
    std::fprintf(stderr, "Motor 0x%02X: starting init\n", node_id_);

    // 1. NMT pre-operational
    nmt_send(sock, canopen::NMT_PRE_OPERATIONAL, node_id_);
    std::this_thread::sleep_for(50ms);

    // 2. Verify factory UID (0x1018 sub 1)
    uint32_t uid = 0;
    SDO_CHECK(sdo_read_u32(sock, node_id_, 0x1018, 1, uid),
              "read factory UID");
    if (uid != HEX_FACTORY_UID) {
        std::fprintf(stderr, "Motor 0x%02X: unknown factory UID 0x%08X\n",
                     node_id_, uid);
        return false;
    }

    // 3. Verify firmware version (0x1018 sub 3)
    SDO_CHECK(sdo_read_u32(sock, node_id_, 0x1018, 3, firmware_version_),
              "read firmware version");
    if (firmware_version_ < HEX_MIN_FW_VERSION) {
        std::fprintf(stderr,
                     "Motor 0x%02X: firmware %u too old (need >= %u)\n",
                     node_id_, firmware_version_, HEX_MIN_FW_VERSION);
        return false;
    }

    // 4. Read peak torque (0x6076 sub 0, f32)
    SDO_CHECK(sdo_read_f32(sock, node_id_, 0x6076, 0, peak_torque_),
              "read peak torque");
    std::fprintf(stderr, "Motor 0x%02X: FW=%u, peak_torque=%.2f Nm\n",
                 node_id_, firmware_version_, peak_torque_);

    // 5. Clear control word, then fault reset
    SDO_CHECK(sdo_write_u16(sock, node_id_, 0x6040, 0, 0x0000), "ctrl=0");
    std::this_thread::sleep_for(10ms);
    SDO_CHECK(sdo_write_u16(sock, node_id_, 0x6040, 0, 0x0080), "fault reset");
    std::this_thread::sleep_for(10ms);

    // 6. Select MIT mode (0x6060 sub 0 = 5)
    SDO_CHECK(sdo_write_i8(sock, node_id_, 0x6060, 0, 5), "MIT mode");
    std::this_thread::sleep_for(10ms);

    // 7. Enable compressed MIT (0x2004 sub 1 = 1)
    SDO_CHECK(sdo_write_u8(sock, node_id_, 0x2004, 0x01, 1), "compressed MIT");
    std::this_thread::sleep_for(10ms);

    // 8. Write initial MIT control values (all zeros encoded)
    MitTarget zero_target{};
    uint8_t encoded[8];
    zero_target.encode(mapping_, encoded);
    uint32_t lo, hi;
    std::memcpy(&lo, encoded,     4);
    std::memcpy(&hi, encoded + 4, 4);

    SDO_CHECK(sdo_write_u32(sock, node_id_, 0x2004, 0x02, lo), "MIT init lo");
    std::this_thread::sleep_for(10ms);
    SDO_CHECK(sdo_write_u32(sock, node_id_, 0x2004, 0x03, hi), "MIT init hi");
    std::this_thread::sleep_for(10ms);

    // 9. CiA 402 state machine: Shutdown → Switch On → Enable Operation
    SDO_CHECK(sdo_write_u16(sock, node_id_, 0x6040, 0, 0x0006), "ctrl=6");
    std::this_thread::sleep_for(10ms);
    SDO_CHECK(sdo_write_u16(sock, node_id_, 0x6040, 0, 0x0007), "ctrl=7");
    std::this_thread::sleep_for(10ms);
    SDO_CHECK(sdo_write_u16(sock, node_id_, 0x6040, 0, 0x000F), "ctrl=0F");
    std::this_thread::sleep_for(10ms);

    // 10. NMT operational
    nmt_send(sock, canopen::NMT_OPERATIONAL, node_id_);

    std::fprintf(stderr, "Motor 0x%02X: init done\n", node_id_);
    return true;
}

#undef SDO_CHECK

// ---------------------------------------------------------------------------
// TPDO parsing
// ---------------------------------------------------------------------------

void HexMotor::process_tpdo1(const uint8_t* data, size_t len) {
    if (len < 12) return;

    // Position: f32, single-turn rev [-0.5, 0.5)
    float pos;
    std::memcpy(&pos, data, 4);

    // Multi-turn tracking
    if (has_prev_pos_) {
        float diff = pos - prev_pos_;
        if (diff > 0.5f)       status_.multi_turn--;
        else if (diff < -0.5f) status_.multi_turn++;
    }
    prev_pos_ = pos;
    has_prev_pos_ = true;
    status_.position_rev = pos;

    // Timestamp (microseconds, u32)
    std::memcpy(&status_.timestamp_us, data + 4, 4);

    // Torque: i16 permille of peak torque → Nm
    int16_t raw_torque;
    std::memcpy(&raw_torque, data + 8, 2);
    status_.torque_nm = static_cast<float>(raw_torque) * peak_torque_ / 1000.0f;

    // Error code: u16
    std::memcpy(&status_.error_code, data + 10, 2);
}

void HexMotor::process_tpdo2(const uint8_t* data, size_t len) {
    if (len < 10) return;

    std::memcpy(&status_.status_word, data, 2);

    int16_t raw;
    std::memcpy(&raw, data + 2, 2);
    status_.driver_temp_c = static_cast<float>(raw) * 0.1f;

    std::memcpy(&raw, data + 4, 2);
    status_.motor_temp_c = static_cast<float>(raw) * 0.1f;

    std::memcpy(&status_.control_word, data + 6, 2);

    if (len >= 12) {
        std::memcpy(&status_.error_code_2, data + 8, 2);
    }
}
