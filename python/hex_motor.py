"""HEX motor abstraction layer.

Mirrors cpp/src/hex_motor.hpp / hex_motor.cpp, adapted for Python.
"""

import struct
import sys
import time
from dataclasses import dataclass, field
from typing import Optional

from canopen import (
    CANBus,
    SDOStatus,
    NMT_OPERATIONAL,
    NMT_PRE_OPERATIONAL,
    nmt_send,
    sdo_read_u32,
    sdo_read_f32,
    sdo_write_u8,
    sdo_write_i8,
    sdo_write_u16,
    sdo_write_u32,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

HEX_FACTORY_UID    = 0x4859444C   # ASCII "HYDL"
HEX_MIN_FW_VERSION = 8
MASTER_NODE_ID     = 0x10


# ---------------------------------------------------------------------------
# MIT target mapping
# ---------------------------------------------------------------------------

@dataclass
class MitTargetMapping:
    """Physical ranges agreed upon by both host and motor firmware.

    If the motor's EEPROM doesn't match these, write them via SDO 0x2004
    sub 4..0x0D before calling HexMotor.init().
    """
    position_min: float = -0.5     # Rev
    position_max: float =  0.5     # Rev
    velocity_min: float = -10.0    # Rev/s
    velocity_max: float =  10.0    # Rev/s
    torque_min:   float = -10.0    # Nm
    torque_max:   float =  10.0    # Nm
    kp_min:       float =  0.0     # Nm/Rev
    kp_max:       float =  100.0   # Nm/Rev
    kd_min:       float =  0.0     # Nm·s/Rev
    kd_max:       float =  20.0    # Nm·s/Rev


# ---------------------------------------------------------------------------
# MIT target
# ---------------------------------------------------------------------------

@dataclass
class MitTarget:
    """Compressed MIT control target.

    Bit layout inside the 8-byte PDO payload (little-endian u64):
      [11:0]  torque   (12 bit)
      [23:12] kd       (12 bit)
      [35:24] kp       (12 bit)
      [47:36] velocity (12 bit)
      [63:48] position (16 bit)
    """
    position: float = 0.0   # Rev
    velocity: float = 0.0   # Rev/s
    torque:   float = 0.0   # Nm
    kp:       float = 0.0   # Nm/Rev
    kd:       float = 0.0   # Nm·s/Rev

    def encode(self, m: MitTargetMapping) -> bytes:
        """Encode this target into 8 bytes suitable for the RPDO payload."""

        def clamp(v: float, lo: float, hi: float) -> float:
            return max(lo, min(hi, v))

        def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
            span  = x_max - x_min
            scale = (1 << bits) - 1
            return int((x - x_min) * scale / span)

        p = clamp(self.position, m.position_min, m.position_max)
        v = clamp(self.velocity, m.velocity_min, m.velocity_max)
        t = clamp(self.torque,   m.torque_min,   m.torque_max)
        k = clamp(self.kp,       m.kp_min,       m.kp_max)
        d = clamp(self.kd,       m.kd_min,       m.kd_max)

        pos_u = float_to_uint(p, m.position_min, m.position_max, 16)
        vel_u = float_to_uint(v, m.velocity_min, m.velocity_max, 12)
        trq_u = float_to_uint(t, m.torque_min,   m.torque_max,   12)
        kp_u  = float_to_uint(k, m.kp_min,       m.kp_max,       12)
        kd_u  = float_to_uint(d, m.kd_min,       m.kd_max,       12)

        lo = (trq_u) | (kd_u << 12) | ((kp_u & 0xFF) << 24)
        hi = (kp_u >> 8) | (vel_u << 4) | (pos_u << 16)

        return struct.pack("<II", lo & 0xFFFFFFFF, hi & 0xFFFFFFFF)


# ---------------------------------------------------------------------------
# Motor status (decoded from TPDO frames)
# ---------------------------------------------------------------------------

@dataclass
class MotorStatus:
    # TPDO1  (~1 kHz)
    position_rev:  float = 0.0    # single-turn [-0.5, 0.5)
    multi_turn:    int   = 0      # accumulated full turns
    timestamp_us:  int   = 0      # on-motor timestamp in µs
    torque_nm:     float = 0.0    # actual torque in Nm
    error_code:    int   = 0

    # TPDO2  (~50 Hz)
    status_word:   int   = 0
    driver_temp_c: float = 0.0
    motor_temp_c:  float = 0.0
    control_word:  int   = 0
    error_code_2:  int   = 0


# ---------------------------------------------------------------------------
# HexMotor
# ---------------------------------------------------------------------------

class HexMotor:
    """Represents one HEX-XXXX motor on the CAN bus."""

    def __init__(self, node_id: int,
                 mapping: Optional[MitTargetMapping] = None) -> None:
        self.node_id:          int              = node_id
        self.mapping:          MitTargetMapping = mapping or MitTargetMapping()
        self.peak_torque:      float            = 0.0
        self.firmware_version: int              = 0
        self.status:           MotorStatus      = MotorStatus()

        self._prev_pos:      float = 0.0
        self._has_prev_pos:  bool  = False

    # ------------------------------------------------------------------
    # Internal helper
    # ------------------------------------------------------------------

    def _sdo_check(self, status: SDOStatus, msg: str) -> bool:
        if status != SDOStatus.OK:
            print(f"Motor 0x{self.node_id:02X}: {msg} ({status.value})",
                  file=sys.stderr)
            return False
        return True

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------

    def init(self, bus: CANBus) -> bool:
        """Run the full SDO initialisation sequence.

        Assumes the following parameters are already saved to EEPROM:
          - PDO mappings (RPDO / TPDO)
          - Heartbeat monitoring (0x1016[1])
          - Short-circuit braking (0x2040[0])
          - Max torque (0x6072[0])

        If any of these have not been saved, configure them first (e.g. via
        canopenlinux or a separate setup script).
        """
        print(f"Motor 0x{self.node_id:02X}: starting init", file=sys.stderr)

        # 1. NMT → pre-operational
        nmt_send(bus, NMT_PRE_OPERATIONAL, self.node_id)
        time.sleep(0.05)

        # 2. Verify factory UID  (0x1018 sub 1)
        st, uid = sdo_read_u32(bus, self.node_id, 0x1018, 1)
        if not self._sdo_check(st, "read factory UID"):
            return False
        if uid != HEX_FACTORY_UID:
            print(f"Motor 0x{self.node_id:02X}: unknown factory UID "
                  f"0x{uid:08X} (expected 0x{HEX_FACTORY_UID:08X})",
                  file=sys.stderr)
            return False

        # 3. Verify firmware version  (0x1018 sub 3)
        st, fw = sdo_read_u32(bus, self.node_id, 0x1018, 3)
        if not self._sdo_check(st, "read firmware version"):
            return False
        self.firmware_version = fw
        if fw < HEX_MIN_FW_VERSION:
            print(f"Motor 0x{self.node_id:02X}: firmware {fw} too old "
                  f"(need >= {HEX_MIN_FW_VERSION})", file=sys.stderr)
            return False

        # 4. Read peak torque  (0x6076 sub 0, f32)
        st, pt = sdo_read_f32(bus, self.node_id, 0x6076, 0)
        if not self._sdo_check(st, "read peak torque"):
            return False
        self.peak_torque = pt
        print(f"Motor 0x{self.node_id:02X}: FW={fw}, "
              f"peak_torque={pt:.2f} Nm", file=sys.stderr)

        # 5. Clear control word, then fault reset
        if not self._sdo_check(
                sdo_write_u16(bus, self.node_id, 0x6040, 0, 0x0000), "ctrl=0"):
            return False
        time.sleep(0.01)
        if not self._sdo_check(
                sdo_write_u16(bus, self.node_id, 0x6040, 0, 0x0080), "fault reset"):
            return False
        time.sleep(0.01)

        # 6. Select MIT mode  (0x6060 sub 0 = 5)
        if not self._sdo_check(
                sdo_write_i8(bus, self.node_id, 0x6060, 0, 5), "MIT mode"):
            return False
        time.sleep(0.01)

        # 7. Enable compressed MIT  (0x2004 sub 1 = 1)
        if not self._sdo_check(
                sdo_write_u8(bus, self.node_id, 0x2004, 0x01, 1), "compressed MIT"):
            return False
        time.sleep(0.01)

        # 8. Write initial (zero) MIT control values
        zero    = MitTarget()
        encoded = zero.encode(self.mapping)
        lo, hi  = struct.unpack_from("<II", encoded)

        if not self._sdo_check(
                sdo_write_u32(bus, self.node_id, 0x2004, 0x02, lo), "MIT init lo"):
            return False
        time.sleep(0.01)
        if not self._sdo_check(
                sdo_write_u32(bus, self.node_id, 0x2004, 0x03, hi), "MIT init hi"):
            return False
        time.sleep(0.01)

        # 9. CiA 402 state machine: Shutdown → Switch On → Enable Operation
        if not self._sdo_check(
                sdo_write_u16(bus, self.node_id, 0x6040, 0, 0x0006), "ctrl=6"):
            return False
        time.sleep(0.01)
        if not self._sdo_check(
                sdo_write_u16(bus, self.node_id, 0x6040, 0, 0x0007), "ctrl=7"):
            return False
        time.sleep(0.01)
        if not self._sdo_check(
                sdo_write_u16(bus, self.node_id, 0x6040, 0, 0x000F), "ctrl=0F"):
            return False
        time.sleep(0.01)

        # 10. NMT → operational
        nmt_send(bus, NMT_OPERATIONAL, self.node_id)

        print(f"Motor 0x{self.node_id:02X}: init done", file=sys.stderr)
        return True

    # ------------------------------------------------------------------
    # TPDO parsing
    # ------------------------------------------------------------------

    def process_tpdo1(self, data: bytes) -> None:
        """Parse TPDO1 frame (12 bytes, ~1 kHz).

        Layout:
          [0..3]  position  f32  single-turn rev in [-0.5, 0.5)
          [4..7]  timestamp u32  µs
          [8..9]  torque    i16  permille of peak torque
          [10..11] error    u16
        """
        if len(data) < 12:
            return

        pos, = struct.unpack_from("<f", data, 0)

        # Multi-turn tracking
        if self._has_prev_pos:
            diff = pos - self._prev_pos
            if diff > 0.5:
                self.status.multi_turn -= 1
            elif diff < -0.5:
                self.status.multi_turn += 1
        self._prev_pos     = pos
        self._has_prev_pos = True
        self.status.position_rev = pos

        self.status.timestamp_us, = struct.unpack_from("<I", data, 4)

        raw_torque, = struct.unpack_from("<h", data, 8)
        self.status.torque_nm = raw_torque * self.peak_torque / 1000.0

        self.status.error_code, = struct.unpack_from("<H", data, 10)

    def process_tpdo2(self, data: bytes) -> None:
        """Parse TPDO2 frame (10–12 bytes, ~50 Hz).

        Layout:
          [0..1]  status_word  u16
          [2..3]  driver_temp  i16  × 0.1 °C
          [4..5]  motor_temp   i16  × 0.1 °C
          [6..7]  control_word u16
          [8..9]  error_code_2 u16  (optional)
        """
        if len(data) < 10:
            return

        self.status.status_word, = struct.unpack_from("<H", data, 0)

        raw, = struct.unpack_from("<h", data, 2)
        self.status.driver_temp_c = raw * 0.1

        raw, = struct.unpack_from("<h", data, 4)
        self.status.motor_temp_c = raw * 0.1

        self.status.control_word, = struct.unpack_from("<H", data, 6)

        if len(data) >= 12:
            self.status.error_code_2, = struct.unpack_from("<H", data, 8)
