#!/usr/bin/env python3
"""HEX Motor Python example — controls two motors via CANopen / MIT mode.

Mirrors cpp/src/main.cpp.

Usage:
    python main.py <can_interface> [motor_id_1] [motor_id_2]
    python main.py can0 1 2

Pre-requisites:
    - The CAN interface must be up and running (e.g. `ip link set can0 up type can bitrate 1000000`).
    - Both motors must have their PDO mappings, heartbeat monitoring,
      short-circuit braking and max-torque limits already saved to EEPROM.
      See README.md for the full list of SDOs to configure.
"""

import signal
import sys
import time

from canopen import (
    CANBus,
    TPDO1_BASE,
    TPDO2_BASE,
    NMT_PRE_OPERATIONAL,
    STATE_OPERATIONAL,
    nmt_send,
    heartbeat_send,
)
from hex_motor import HexMotor, MitTarget, MASTER_NODE_ID

# RPDO COB-ID that all motors listen on (pre-configured via PDO mapping).
# Default: TPDO1 function code (0x180) | MASTER_NODE_ID (0x10) = 0x190.
RPDO_COB_ID = TPDO1_BASE | MASTER_NODE_ID

_running = True


def _on_signal(sig: int, frame: object) -> None:
    global _running
    _running = False


def print_motor(motor: HexMotor) -> None:
    s = motor.status
    total_rev = s.multi_turn + s.position_rev
    print(
        f"  Motor 0x{motor.node_id:02X} | "
        f"pos {total_rev:+8.4f} rev ({s.multi_turn:+d} turns) | "
        f"torque {s.torque_nm:+6.3f} Nm | "
        f"err 0x{s.error_code:04X} | "
        f"Td {s.driver_temp_c:.1f}°C  Tm {s.motor_temp_c:.1f}°C"
    )


def main() -> None:
    if len(sys.argv) < 2:
        print(
            f"Usage: {sys.argv[0]} <can_interface> [motor_id_1] [motor_id_2]\n"
            f"  Example: {sys.argv[0]} can0 1 2",
            file=sys.stderr,
        )
        sys.exit(1)

    iface = sys.argv[1]
    id1   = int(sys.argv[2]) if len(sys.argv) > 2 else 0x01
    id2   = int(sys.argv[3]) if len(sys.argv) > 3 else 0x02

    bus = CANBus()
    if not bus.open(iface):
        sys.exit(1)

    print(f"Opened CAN interface {iface}")

    motor1 = HexMotor(id1)
    motor2 = HexMotor(id2)

    if not motor1.init(bus):
        print(f"Failed to initialize motor 0x{id1:02X}", file=sys.stderr)
        bus.close()
        sys.exit(1)

    if not motor2.init(bus):
        print(f"Failed to initialize motor 0x{id2:02X}", file=sys.stderr)
        bus.close()
        sys.exit(1)

    signal.signal(signal.SIGINT,  _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    last_hb     = time.monotonic()
    last_target = time.monotonic()
    last_print  = time.monotonic()

    print("Running (Ctrl-C to stop)...")

    while _running:
        now = time.monotonic()

        # Send master heartbeat every 50 ms
        if now - last_hb >= 0.05:
            heartbeat_send(bus, MASTER_NODE_ID, STATE_OPERATIONAL)
            last_hb = now

        # Send MIT targets every 5 ms (200 Hz) via CAN FD with BRS
        if now - last_target >= 0.005:
            t1 = MitTarget()
            t2 = MitTarget()

            # -----------------------------------------------------------
            # Set your desired targets here. All zeros = hold position
            # with zero torque feed-forward and zero gains.
            #
            # Example: pure torque of 0.1 Nm on motor 1:
            #   t1.torque = 0.1
            #
            # Example: position hold at 0 with stiffness on motor 2:
            #   t2.kp = 5.0
            #   t2.kd = 0.5
            # -----------------------------------------------------------
            t1.velocity = 1.0
            t1.kd       = 1.0
            t2.velocity = 2.0
            t2.kd       = 1.0

            pdo_data = t1.encode(motor1.mapping) + t2.encode(motor2.mapping)
            bus.send_canfd(RPDO_COB_ID, pdo_data)
            last_target = now

        # Drain all pending incoming CAN frames
        while True:
            msg = bus.recv(timeout=0)
            if msg is None:
                break

            cob_id = msg.arbitration_id & 0x7FF

            if   cob_id == TPDO1_BASE + id1:
                motor1.process_tpdo1(bytes(msg.data))
            elif cob_id == TPDO1_BASE + id2:
                motor2.process_tpdo1(bytes(msg.data))
            elif cob_id == TPDO2_BASE + id1:
                motor1.process_tpdo2(bytes(msg.data))
            elif cob_id == TPDO2_BASE + id2:
                motor2.process_tpdo2(bytes(msg.data))

        # Print status every 500 ms
        if now - last_print >= 0.5:
            print_motor(motor1)
            print_motor(motor2)
            print()
            last_print = now

        time.sleep(0.001)

    print("\nShutting down...")
    nmt_send(bus, NMT_PRE_OPERATIONAL, 0x00)
    bus.close()


if __name__ == "__main__":
    main()
