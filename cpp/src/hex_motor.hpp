#pragma once

#include "canopen.hpp"
#include <cstdint>
#include <cmath>
#include <algorithm>

constexpr uint32_t HEX_FACTORY_UID    = 0x4859444C;
constexpr uint32_t HEX_MIN_FW_VERSION = 8;
constexpr uint8_t  MASTER_NODE_ID     = 0x10;

// MIT compressed target mapping (min/max per field).
// Both the host and motor must agree on these values.
// Use sdo_write to 0x2004[4..0x0D] if they haven't been saved.
struct MitTargetMapping {
    float position_min = -0.5f;   // Rev
    float position_max =  0.5f;
    float velocity_min = -10.0f;  // Rev/s
    float velocity_max =  10.0f;
    float torque_min   = -10.0f;  // Nm
    float torque_max   =  10.0f;
    float kp_min       =  0.0f;   // Nm/Rev
    float kp_max       =  100.0f;
    float kd_min       =  0.0f;   // Nm·s/Rev
    float kd_max       =  20.0f;
};

// Compressed MIT target (all units in Rev, Rev/s, Nm).
struct MitTarget {
    float position = 0.0f;
    float velocity = 0.0f;
    float torque   = 0.0f;
    float kp       = 0.0f;
    float kd       = 0.0f;

    // Encode into 8 bytes matching the motor's compressed MIT format.
    //   Bit layout (little-endian u64):
    //     [11:0]  torque  (12 bit)
    //     [23:12] kd      (12 bit)
    //     [35:24] kp      (12 bit)
    //     [47:36] velocity(12 bit)
    //     [63:48] position(16 bit)
    void encode(const MitTargetMapping& m, uint8_t out[8]) const;
};

struct MotorStatus {
    // TPDO1 (12 bytes, ~1 kHz)
    float    position_rev    = 0.0f;  // single-turn [-0.5, 0.5)
    int32_t  multi_turn      = 0;     // accumulated full turns
    uint32_t timestamp_us    = 0;
    float    torque_nm       = 0.0f;
    uint16_t error_code      = 0;

    // TPDO2 (10–12 bytes, ~50 Hz)
    uint16_t status_word     = 0;
    float    driver_temp_c   = 0.0f;
    float    motor_temp_c    = 0.0f;
    uint16_t control_word    = 0;
    uint16_t error_code_2    = 0;
};

class HexMotor {
public:
    explicit HexMotor(uint8_t node_id,
                      const MitTargetMapping& mapping = MitTargetMapping{});

    // Full initialization sequence via SDO.
    // Pre-saved parameters (PDO mappings, heartbeat monitoring, short-circuit
    // braking, max torque) are assumed to already be stored on the motor.
    bool init(CANSocket& sock);

    void process_tpdo1(const uint8_t* data, size_t len);
    void process_tpdo2(const uint8_t* data, size_t len);

    const MotorStatus&      status()      const { return status_; }
    uint8_t                 node_id()     const { return node_id_; }
    float                   peak_torque() const { return peak_torque_; }
    const MitTargetMapping& mapping()     const { return mapping_; }

private:
    uint8_t          node_id_;
    MitTargetMapping mapping_;
    float            peak_torque_      = 0.0f;
    uint32_t         firmware_version_ = 0;
    MotorStatus      status_;
    float            prev_pos_         = 0.0f;
    bool             has_prev_pos_     = false;
};
