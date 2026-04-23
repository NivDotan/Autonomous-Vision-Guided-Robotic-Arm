#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>

// ── Command codes ─────────────────────────────────────────────────────────────
// All ZeroMQ messages are msgpack-encoded maps with a "cmd" key.
enum class DaemonCmd : uint8_t {
    WRITE_TICKS    = 0x01,
    READ_TICKS     = 0x02,
    GRIPPER_LOAD   = 0x03,
    SET_PID        = 0x04,
    SET_TRAJECTORY = 0x05,
    STATUS         = 0xFF,
};

// ── Response status ──────────────────────────────────────────────────────────
enum class DaemonStatus : uint8_t {
    OK    = 0,
    ERROR = 1,
};

// ── Joint ordering ────────────────────────────────────────────────────────────
// Index 0=base, 1=shoulder, 2=elbow, 3=palm, 4=wrist, 5=gripper
constexpr int NUM_JOINTS = 6;
using Ticks6 = std::array<int16_t, NUM_JOINTS>;

// ── PID parameters ────────────────────────────────────────────────────────────
struct PIDParams {
    int    joint;
    double kp;
    double ki;
    double kd;
    double i_max;
};

// ── Trajectory waypoint (compact, for wire format) ───────────────────────────
struct WireWaypoint {
    float   t_ms;   // time in milliseconds from trajectory start
    Ticks6  ticks;
};

// ── Default serial port ───────────────────────────────────────────────────────
#ifdef _WIN32
static constexpr const char* DEFAULT_PORT = "COM4";
#else
static constexpr const char* DEFAULT_PORT = "/dev/ttyUSB0";
#endif

static constexpr int    BAUD_RATE    = 1000000;  // 1 Mbit/s (Feetech STS3215)
static constexpr int    CONTROL_HZ   = 200;       // control loop frequency
static constexpr double CONTROL_DT   = 1.0 / CONTROL_HZ;
static constexpr int    ZMQ_PORT     = 5555;
