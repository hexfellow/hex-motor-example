"""CANopen protocol helpers using python-can.

Mirrors cpp/src/canopen.hpp / canopen.cpp, adapted for Python.
"""

import struct
import time
from enum import Enum
from typing import Optional, Tuple

import can

# ---------------------------------------------------------------------------
# CANopen COB-ID bases
# ---------------------------------------------------------------------------

NMT_ID         = 0x000
TPDO1_BASE     = 0x180
RPDO1_BASE     = 0x200
TPDO2_BASE     = 0x280
SDO_TX_BASE    = 0x580
SDO_RX_BASE    = 0x600
HEARTBEAT_BASE = 0x700

# ---------------------------------------------------------------------------
# NMT commands
# ---------------------------------------------------------------------------

NMT_OPERATIONAL     = 0x01
NMT_STOPPED         = 0x02
NMT_PRE_OPERATIONAL = 0x80
NMT_RESET_NODE      = 0x81
NMT_RESET_COMM      = 0x82

# Node states (reported in heartbeat)
STATE_OPERATIONAL     = 0x05
STATE_PRE_OPERATIONAL = 0x7F


# ---------------------------------------------------------------------------
# SDO status
# ---------------------------------------------------------------------------

class SDOStatus(Enum):
    OK      = "OK"
    Timeout = "Timeout"
    Aborted = "Aborted"
    Error   = "Error"


# ---------------------------------------------------------------------------
# CANBus wrapper
# ---------------------------------------------------------------------------

class CANBus:
    """Thin wrapper around python-can Bus, mirroring CANSocket in C++."""

    def __init__(self) -> None:
        self._bus: Optional[can.BusABC] = None

    def open(self, interface_name: str, bustype: str = "socketcan") -> bool:
        try:
            self._bus = can.Bus(channel=interface_name, interface=bustype, fd=True)
            return True
        except Exception as exc:
            print(f"Failed to open CAN interface '{interface_name}': {exc}")
            return False

    def close(self) -> None:
        if self._bus is not None:
            self._bus.shutdown()
            self._bus = None

    def is_open(self) -> bool:
        return self._bus is not None

    def send_can(self, can_id: int, data: bytes) -> bool:
        """Send a classic CAN frame (≤ 8 bytes)."""
        if self._bus is None:
            return False
        try:
            msg = can.Message(
                arbitration_id=can_id,
                data=data,
                is_extended_id=False,
                is_fd=False,
            )
            self._bus.send(msg)
            return True
        except Exception as exc:
            print(f"send_can error: {exc}")
            return False

    def send_canfd(self, can_id: int, data: bytes, brs: bool = True) -> bool:
        """Send a CAN FD frame (≤ 64 bytes)."""
        if self._bus is None:
            return False
        try:
            msg = can.Message(
                arbitration_id=can_id,
                data=data,
                is_extended_id=False,
                is_fd=True,
                bitrate_switch=brs,
            )
            self._bus.send(msg)
            return True
        except Exception as exc:
            print(f"send_canfd error: {exc}")
            return False

    def recv(self, timeout: float) -> Optional[can.Message]:
        """Receive one frame; returns None on timeout or error."""
        if self._bus is None:
            return None
        return self._bus.recv(timeout=timeout)


# ---------------------------------------------------------------------------
# SDO helpers
# ---------------------------------------------------------------------------

def _wait_sdo_response(bus: CANBus, node_id: int, timeout: float) -> Optional[bytes]:
    """Block until an SDO response from *node_id* arrives or timeout expires."""
    expected_id = SDO_TX_BASE + node_id
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        msg = bus.recv(timeout=max(remaining, 0))
        if msg is None:
            return None
        if msg.arbitration_id == expected_id and len(msg.data) >= 8:
            return bytes(msg.data[:8])
    return None


def sdo_read(
    bus: CANBus,
    node_id: int,
    index: int,
    sub: int,
    timeout: float = 1.0,
) -> Tuple[SDOStatus, bytes]:
    """Expedited SDO upload (read from node)."""
    req = struct.pack("<BBBBBBBB", 0x40, index & 0xFF, (index >> 8) & 0xFF, sub,
                     0, 0, 0, 0)
    if not bus.send_can(SDO_RX_BASE + node_id, req):
        return SDOStatus.Error, b""

    resp = _wait_sdo_response(bus, node_id, timeout)
    if resp is None:
        return SDOStatus.Timeout, b""

    if resp[0] == 0x80:
        return SDOStatus.Aborted, b""

    cmd = resp[0]
    data_len = 4
    if cmd & 0x01:                  # size indicated
        n = (cmd >> 2) & 0x03
        data_len = 4 - n

    return SDOStatus.OK, resp[4: 4 + data_len]


def sdo_write(
    bus: CANBus,
    node_id: int,
    index: int,
    sub: int,
    data: bytes,
    timeout: float = 1.0,
) -> SDOStatus:
    """Expedited SDO download (write to node)."""
    size = len(data)
    if size == 0 or size > 4:
        return SDOStatus.Error

    n   = 4 - size
    cmd = 0x20 | (n << 2) | 0x03   # expedited + size indicated

    req = bytearray(8)
    req[0] = cmd
    req[1] = index & 0xFF
    req[2] = (index >> 8) & 0xFF
    req[3] = sub
    req[4: 4 + size] = data

    if not bus.send_can(SDO_RX_BASE + node_id, bytes(req)):
        return SDOStatus.Error

    resp = _wait_sdo_response(bus, node_id, timeout)
    if resp is None:
        return SDOStatus.Timeout
    if resp[0] == 0x80:
        return SDOStatus.Aborted
    if resp[0] != 0x60:
        return SDOStatus.Error

    return SDOStatus.OK


# ---------------------------------------------------------------------------
# Typed convenience wrappers
# ---------------------------------------------------------------------------

def sdo_read_u32(bus: CANBus, node_id: int, index: int, sub: int
                 ) -> Tuple[SDOStatus, int]:
    st, raw = sdo_read(bus, node_id, index, sub)
    if st != SDOStatus.OK:
        return st, 0
    padded = raw.ljust(4, b"\x00")
    return SDOStatus.OK, struct.unpack_from("<I", padded)[0]


def sdo_read_f32(bus: CANBus, node_id: int, index: int, sub: int
                 ) -> Tuple[SDOStatus, float]:
    st, raw = sdo_read(bus, node_id, index, sub)
    if st != SDOStatus.OK:
        return st, 0.0
    padded = raw.ljust(4, b"\x00")
    return SDOStatus.OK, struct.unpack_from("<f", padded)[0]


def sdo_write_u8(bus: CANBus, node_id: int, index: int, sub: int,
                 value: int) -> SDOStatus:
    return sdo_write(bus, node_id, index, sub, struct.pack("<B", value & 0xFF))


def sdo_write_i8(bus: CANBus, node_id: int, index: int, sub: int,
                 value: int) -> SDOStatus:
    return sdo_write(bus, node_id, index, sub, struct.pack("<b", value))


def sdo_write_u16(bus: CANBus, node_id: int, index: int, sub: int,
                  value: int) -> SDOStatus:
    return sdo_write(bus, node_id, index, sub, struct.pack("<H", value & 0xFFFF))


def sdo_write_u32(bus: CANBus, node_id: int, index: int, sub: int,
                  value: int) -> SDOStatus:
    return sdo_write(bus, node_id, index, sub, struct.pack("<I", value & 0xFFFFFFFF))


# ---------------------------------------------------------------------------
# NMT / Heartbeat
# ---------------------------------------------------------------------------

def nmt_send(bus: CANBus, command: int, node_id: int) -> bool:
    """Send an NMT command frame.  node_id=0 targets all nodes."""
    return bus.send_can(NMT_ID, bytes([command, node_id]))


def heartbeat_send(bus: CANBus, node_id: int, state: int) -> bool:
    """Send the master heartbeat on HEARTBEAT_BASE + node_id."""
    return bus.send_can(HEARTBEAT_BASE + node_id, bytes([state]))
