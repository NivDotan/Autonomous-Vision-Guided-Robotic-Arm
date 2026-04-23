#pragma once
#include "daemon_protocol.hpp"
#include <string>
#include <vector>

#ifdef _WIN32
#  include <windows.h>
#endif

class SerialComm {
public:
    SerialComm();
    ~SerialComm();

    // Open real serial port (Feetech protocol at baud_rate).
    bool open(const std::string& port, int baud);

    // Open in simulation mode — reads/writes go to an in-memory array.
    bool open_sim();

    void close();

    bool is_open() const {
#ifdef _WIN32
        return sim_mode_ || handle_ != INVALID_HANDLE_VALUE;
#else
        return sim_mode_ || fd_ >= 0;
#endif
    }

    // Enable torque on all motors (must call after open()).
    void torque_enable_all(const std::array<int, 6>& ids, bool enable = true);

    // Bulk sync-read Present_Position from all motors.
    bool sync_read_positions(const std::array<int, 6>& ids, Ticks6& out);

    // Bulk sync-write Goal_Position to all motors.
    bool sync_write_positions(const std::array<int, 6>& ids, const Ticks6& ticks);

    // Read Present_Load for one motor (gripper = motor_id 6).
    int16_t read_load(int motor_id);

private:
    bool sim_mode_;
    Ticks6   sim_pos_;
    std::array<int16_t, 6> sim_load_;

#ifdef _WIN32
    HANDLE handle_;
#else
    int fd_;
#endif
};
