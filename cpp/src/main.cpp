#include "canopen.hpp"
#include "hex_motor.hpp"

#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <chrono>
#include <thread>

using clk = std::chrono::steady_clock;
using namespace std::chrono_literals;

static volatile sig_atomic_t g_running = 1;

static void on_signal(int) { g_running = 0; }

static void print_motor(const HexMotor& m) {
    const auto& s = m.status();
    float total_rev = s.multi_turn + s.position_rev;
    std::printf("  Motor 0x%02X | pos %+8.4f rev (%+d turns) | "
                "torque %+6.3f Nm | err 0x%04X | "
                "Td %.1f°C  Tm %.1f°C\n",
                m.node_id(), total_rev, s.multi_turn,
                s.torque_nm, s.error_code,
                s.driver_temp_c, s.motor_temp_c);
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::fprintf(stderr,
            "Usage: %s <can_interface> [motor_id_1] [motor_id_2]\n"
            "  Example: %s can0 1 2\n",
            argv[0], argv[0]);
        return 1;
    }

    const char* iface = argv[1];
    uint8_t id1 = (argc > 2) ? static_cast<uint8_t>(std::atoi(argv[2])) : 0x01;
    uint8_t id2 = (argc > 3) ? static_cast<uint8_t>(std::atoi(argv[3])) : 0x02;

    CANSocket sock;
    if (!sock.open(iface)) return 1;

    std::printf("Opened CAN interface %s\n", iface);

    HexMotor motor1(id1);
    HexMotor motor2(id2);

    if (!motor1.init(sock)) {
        std::fprintf(stderr, "Failed to initialize motor 0x%02X\n", id1);
        return 1;
    }
    if (!motor2.init(sock)) {
        std::fprintf(stderr, "Failed to initialize motor 0x%02X\n", id2);
        return 1;
    }

    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    // The RPDO COB-ID that all motors listen on (pre-configured via PDO mapping).
    // Default: TPDO1 function code (0x180) | MASTER_NODE_ID (0x10) = 0x190.
    const uint16_t rpdo_cob_id = canopen::TPDO1_BASE | MASTER_NODE_ID;

    auto last_hb     = clk::now();
    auto last_target  = clk::now();
    auto last_print   = clk::now();

    std::printf("Running (Ctrl-C to stop)...\n");

    while (g_running) {
        auto now = clk::now();

        // Send master heartbeat every 50 ms
        if (now - last_hb >= 50ms) {
            heartbeat_send(sock, MASTER_NODE_ID, canopen::STATE_OPERATIONAL);
            last_hb = now;
        }

        // Send MIT targets every 5 ms (200 Hz) via CAN FD with BRS
        if (now - last_target >= 5ms) {
            MitTarget t1{}, t2{};
            // -----------------------------------------------------------
            // Set your desired targets here. All zeros = hold position
            // with zero torque feed-forward and zero gains.
            // Example: pure torque of 0.1 Nm on motor 1
            //   t1.torque = 0.1f;
            // Example: position hold at 0 with stiffness on motor 2
            //   t2.kp = 5.0f;  t2.kd = 0.5f;
            // -----------------------------------------------------------

            uint8_t pdo_data[16];
            t1.encode(motor1.mapping(), pdo_data);
            t2.encode(motor2.mapping(), pdo_data + 8);
            sock.send_canfd(rpdo_cob_id, pdo_data, sizeof(pdo_data));
            last_target = now;
        }

        // Drain incoming CAN frames
        struct canfd_frame frame{};
        while (sock.recv(frame, 0)) {
            uint16_t id = frame.can_id & CAN_SFF_MASK;

            if      (id == canopen::TPDO1_BASE + id1)
                motor1.process_tpdo1(frame.data, frame.len);
            else if (id == canopen::TPDO1_BASE + id2)
                motor2.process_tpdo1(frame.data, frame.len);
            else if (id == canopen::TPDO2_BASE + id1)
                motor1.process_tpdo2(frame.data, frame.len);
            else if (id == canopen::TPDO2_BASE + id2)
                motor2.process_tpdo2(frame.data, frame.len);
        }

        // Print status every 500 ms
        if (now - last_print >= 500ms) {
            print_motor(motor1);
            print_motor(motor2);
            std::printf("\n");
            last_print = now;
        }

        std::this_thread::sleep_for(1ms);
    }

    std::printf("\nShutting down...\n");

    // Return motors to pre-operational
    nmt_send(sock, canopen::NMT_PRE_OPERATIONAL, 0x00);

    sock.close();
    return 0;
}
