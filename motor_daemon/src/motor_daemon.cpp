// motor_daemon.cpp — Real-time 200Hz motor control daemon for SO-101 robot arm.
//
// Architecture:
//   main thread:   ZeroMQ REP socket — receives commands, updates shared state.
//   control thread: 200Hz loop — sync-reads motors, runs PID, writes targets.
//
// Usage:
//   motor_daemon [--port COM4] [--sim] [--zmq-port 5555]

#include "daemon_protocol.hpp"
#include "pid_controller.hpp"
#include "serial_comm.hpp"
#include "trajectory_tracker.hpp"

#include <zmq.h>
#include <msgpack.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>

// ── Shared state (protected by g_mutex) ─────────────────────────────────────
static std::mutex          g_mutex;
static Ticks6              g_target{};
static Ticks6              g_current{};
static int16_t             g_gripper_load  = 0;
static bool                g_gripper_catch = false;
static double              g_loop_hz       = 0.0;
static std::atomic<bool>   g_running{true};

// Motor IDs: base=1, shoulder=2, elbow=3, palm=4, wrist=5, gripper=6
static const std::array<int, 6> MOTOR_IDS = {1, 2, 3, 4, 5, 6};
static constexpr int GRIPPER_IDX = 5;
static constexpr int GRIP_LOAD_THRESHOLD = 150;

// ── PID controllers (one per joint) ─────────────────────────────────────────
static std::array<PIDController, 6> g_pid = {
    PIDController(2.0, 0.1, 0.05, 500.0),  // base
    PIDController(2.0, 0.1, 0.05, 500.0),  // shoulder
    PIDController(2.0, 0.1, 0.05, 500.0),  // elbow
    PIDController(2.0, 0.1, 0.05, 500.0),  // palm
    PIDController(2.0, 0.1, 0.05, 500.0),  // wrist
    PIDController(1.5, 0.05, 0.02, 300.0), // gripper
};

// ── Trajectory tracker ───────────────────────────────────────────────────────
static TrajectoryTracker g_traj;

// ── Control loop thread ──────────────────────────────────────────────────────
static void control_loop(SerialComm& serial) {
    using clock = std::chrono::steady_clock;
    using ns    = std::chrono::nanoseconds;

    const auto period = ns(static_cast<long long>(1e9 / CONTROL_HZ));
    auto next_tick    = clock::now() + period;

    long loop_count = 0;
    auto hz_timer   = clock::now();

    while (g_running) {
        auto now = clock::now();
        double now_ms = std::chrono::duration<double, std::milli>(
                            now.time_since_epoch()).count();

        // 1. Read current positions.
        Ticks6 current_read;
        serial.sync_read_positions(MOTOR_IDS, current_read);

        // 2. Update shared current ticks.
        {
            std::lock_guard<std::mutex> lk(g_mutex);
            g_current = current_read;
        }

        // 3. Get target (from trajectory or static command).
        Ticks6 target_local;
        {
            std::lock_guard<std::mutex> lk(g_mutex);
            if (g_traj.active()) {
                target_local = g_traj.tick(now_ms);
                g_target     = target_local;
            } else {
                target_local = g_target;
            }
        }

        // 4. Write target positions directly.
        // STS3215 has its own internal position PID — we just tell it where to go.
        // Clamp to valid tick range.
        Ticks6 goal;
        for (int i = 0; i < NUM_JOINTS; ++i) {
            goal[i] = static_cast<int16_t>(
                std::clamp(static_cast<int>(target_local[i]), 0, 4095));
        }

        // 5. Write goal positions.
        serial.sync_write_positions(MOTOR_IDS, goal);

        // 6. Read gripper load and detect catch.
        int16_t load = serial.read_load(MOTOR_IDS[GRIPPER_IDX]);
        {
            std::lock_guard<std::mutex> lk(g_mutex);
            g_gripper_load  = load;
            g_gripper_catch = (load > GRIP_LOAD_THRESHOLD);
        }

        // 7. Track loop frequency.
        ++loop_count;
        auto elapsed_hz = std::chrono::duration<double>(clock::now() - hz_timer).count();
        if (elapsed_hz >= 1.0) {
            std::lock_guard<std::mutex> lk(g_mutex);
            g_loop_hz   = loop_count / elapsed_hz;
            loop_count  = 0;
            hz_timer    = clock::now();
        }

        // 8. Sleep until next period (busy-wait last ~100µs for precision).
        std::this_thread::sleep_until(next_tick - std::chrono::microseconds(200));
        while (clock::now() < next_tick) {}  // spin for last bit
        next_tick += period;
    }
}

// ── ZeroMQ message handling ──────────────────────────────────────────────────

static std::vector<uint8_t> handle_message(const uint8_t* data, size_t size) {
    msgpack::object_handle oh = msgpack::unpack(
        reinterpret_cast<const char*>(data), size);
    msgpack::object obj = oh.get();

    std::map<std::string, msgpack::object> req;
    obj.convert(req);

    uint8_t cmd = req.at("cmd").as<uint8_t>();
    msgpack::sbuffer sbuf;
    msgpack::packer<msgpack::sbuffer> pk(sbuf);

    switch (static_cast<DaemonCmd>(cmd)) {
        case DaemonCmd::WRITE_TICKS: {
            auto ticks_vec = req.at("ticks").as<std::vector<int16_t>>();
            std::lock_guard<std::mutex> lk(g_mutex);
            for (int i = 0; i < NUM_JOINTS && i < (int)ticks_vec.size(); ++i) {
                g_target[i] = ticks_vec[i];
            }
            pk.pack_map(2);
            pk.pack(std::string("status")); pk.pack(0);
            pk.pack(std::string("cmd"));    pk.pack(cmd);
            break;
        }
        case DaemonCmd::READ_TICKS: {
            Ticks6 curr;
            { std::lock_guard<std::mutex> lk(g_mutex); curr = g_current; }
            std::vector<int16_t> v(curr.begin(), curr.end());
            pk.pack_map(2);
            pk.pack(std::string("status")); pk.pack(0);
            pk.pack(std::string("ticks"));  pk.pack(v);
            break;
        }
        case DaemonCmd::GRIPPER_LOAD: {
            int16_t load; bool catch_flag;
            {
                std::lock_guard<std::mutex> lk(g_mutex);
                load       = g_gripper_load;
                catch_flag = g_gripper_catch;
            }
            pk.pack_map(3);
            pk.pack(std::string("status"));   pk.pack(0);
            pk.pack(std::string("load"));     pk.pack(load);
            pk.pack(std::string("detected")); pk.pack(catch_flag);
            break;
        }
        case DaemonCmd::SET_PID: {
            int    joint = req.at("joint").as<int>();
            double kp    = req.at("kp").as<double>();
            double ki    = req.at("ki").as<double>();
            double kd    = req.at("kd").as<double>();
            double imax  = req.at("i_max").as<double>();
            if (joint >= 0 && joint < NUM_JOINTS) {
                g_pid[joint].set_gains(kp, ki, kd, imax);
            }
            pk.pack_map(1);
            pk.pack(std::string("status")); pk.pack(0);
            break;
        }
        case DaemonCmd::SET_TRAJECTORY: {
            auto wps_raw = req.at("waypoints")
                               .as<std::vector<std::map<std::string, msgpack::object>>>();
            std::vector<WireWaypoint> wps;
            wps.reserve(wps_raw.size());
            for (auto& wp_map : wps_raw) {
                WireWaypoint wp;
                wp.t_ms = wp_map.at("t").as<float>();
                auto tv = wp_map.at("ticks").as<std::vector<int16_t>>();
                for (int i = 0; i < NUM_JOINTS && i < (int)tv.size(); ++i) {
                    wp.ticks[i] = tv[i];
                }
                wps.push_back(wp);
            }
            g_traj.load(wps);
            pk.pack_map(2);
            pk.pack(std::string("status")); pk.pack(0);
            pk.pack(std::string("count"));  pk.pack(static_cast<int>(wps.size()));
            break;
        }
        case DaemonCmd::STATUS: {
            double hz; bool traj_active;
            Ticks6 curr, tgt;
            {
                std::lock_guard<std::mutex> lk(g_mutex);
                hz          = g_loop_hz;
                traj_active = g_traj.active();
                curr        = g_current;
                tgt         = g_target;
            }
            std::vector<int16_t> cv(curr.begin(), curr.end());
            std::vector<int16_t> tv(tgt.begin(),  tgt.end());
            pk.pack_map(5);
            pk.pack(std::string("status"));            pk.pack(0);
            pk.pack(std::string("loop_hz"));           pk.pack(hz);
            pk.pack(std::string("trajectory_active")); pk.pack(traj_active);
            pk.pack(std::string("current_ticks"));     pk.pack(cv);
            pk.pack(std::string("target_ticks"));      pk.pack(tv);
            break;
        }
        default: {
            pk.pack_map(2);
            pk.pack(std::string("status")); pk.pack(1);
            pk.pack(std::string("msg"));
            pk.pack(std::string("Unknown command: " + std::to_string(cmd)));
        }
    }

    return std::vector<uint8_t>(sbuf.data(), sbuf.data() + sbuf.size());
}

// ── Signal handler ────────────────────────────────────────────────────────────
static void sig_handler(int) { g_running = false; }

// ── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    std::string port      = DEFAULT_PORT;
    int         zmq_port  = ZMQ_PORT;
    bool        sim_mode  = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--sim") {
            sim_mode = true;
        } else if (arg == "--port" && i + 1 < argc) {
            port = argv[++i];
        } else if (arg == "--zmq-port" && i + 1 < argc) {
            zmq_port = std::stoi(argv[++i]);
        }
    }

    // Initialise serial.
    SerialComm serial;
    if (sim_mode) {
        serial.open_sim();
        std::printf("[motor_daemon] Simulation mode — no hardware required.\n");
    } else {
        if (!serial.open(port, BAUD_RATE)) {
            std::fprintf(stderr, "[motor_daemon] Failed to open %s\n", port.c_str());
            return 1;
        }
        std::printf("[motor_daemon] Opened %s @ %d baud\n", port.c_str(), BAUD_RATE);
        serial.torque_enable_all(MOTOR_IDS, true);
        std::printf("[motor_daemon] Torque enabled on all motors.\n");
    }

    // Read current positions and use them as initial targets so the arm
    // stays still until Python sends the first command.
    if (!sim_mode) {
        Ticks6 init_pos{};
        serial.sync_read_positions(MOTOR_IDS, init_pos);
        g_target = init_pos;
        std::printf("[motor_daemon] Initial positions: %d %d %d %d %d %d\n",
            init_pos[0], init_pos[1], init_pos[2],
            init_pos[3], init_pos[4], init_pos[5]);
    } else {
        g_target = {2048, 2048, 2048, 2048, 3200, 3000};
    }

    // Start control thread.
    std::thread ctrl_thread(control_loop, std::ref(serial));

    // ZeroMQ setup.
    void* ctx    = zmq_ctx_new();
    void* socket = zmq_socket(ctx, ZMQ_REP);
    std::string addr = "tcp://*:" + std::to_string(zmq_port);
    zmq_bind(socket, addr.c_str());
    std::printf("[motor_daemon] ZeroMQ REP listening on %s\n", addr.c_str());

    std::signal(SIGINT,  sig_handler);
    std::signal(SIGTERM, sig_handler);

    // Message loop.
    while (g_running) {
        zmq_msg_t msg;
        zmq_msg_init(&msg);
        int rc = zmq_msg_recv(&msg, socket, ZMQ_DONTWAIT);
        if (rc < 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            zmq_msg_close(&msg);
            continue;
        }

        auto* data = static_cast<const uint8_t*>(zmq_msg_data(&msg));
        size_t size = zmq_msg_size(&msg);

        std::vector<uint8_t> reply;
        try {
            reply = handle_message(data, size);
        } catch (const std::exception& e) {
            msgpack::sbuffer sbuf;
            msgpack::packer<msgpack::sbuffer> pk(sbuf);
            pk.pack_map(2);
            pk.pack(std::string("status")); pk.pack(1);
            pk.pack(std::string("msg"));    pk.pack(std::string(e.what()));
            reply.assign(sbuf.data(), sbuf.data() + sbuf.size());
        }

        zmq_msg_close(&msg);
        zmq_send(socket, reply.data(), reply.size(), 0);
    }

    // Cleanup.
    g_running = false;
    ctrl_thread.join();
    zmq_close(socket);
    zmq_ctx_destroy(ctx);
    std::printf("[motor_daemon] Shutdown complete.\n");
    return 0;
}
