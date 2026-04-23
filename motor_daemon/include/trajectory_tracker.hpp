#pragma once
#include "daemon_protocol.hpp"
#include <mutex>
#include <vector>

class TrajectoryTracker {
public:
    TrajectoryTracker();

    // Load a new trajectory (replaces any current one).
    void load(const std::vector<WireWaypoint>& waypoints);

    // Abort the current trajectory.
    void stop();

    bool active() const;

    // Called each control loop tick. Returns target ticks for now_ms.
    Ticks6 tick(double now_ms);

private:
    mutable std::mutex          mutex_;
    std::vector<WireWaypoint>   waypoints_;
    bool                        active_;
    bool                        initialized_;
    double                      start_time_ms_;
    int                         current_index_;
};
