#include "trajectory_tracker.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

// ── TrajectoryTracker ─────────────────────────────────────────────────────────

TrajectoryTracker::TrajectoryTracker()
    : active_(false), start_time_ms_(0.0), current_index_(0)
{}

void TrajectoryTracker::load(const std::vector<WireWaypoint>& waypoints) {
    std::lock_guard<std::mutex> lk(mutex_);
    waypoints_   = waypoints;
    active_      = !waypoints.empty();
    current_index_ = 0;
    start_time_ms_ = 0.0;  // reset: caller sets start time on first tick
    initialized_   = false;
}

void TrajectoryTracker::stop() {
    std::lock_guard<std::mutex> lk(mutex_);
    active_ = false;
    waypoints_.clear();
}

bool TrajectoryTracker::active() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return active_;
}

// Returns the target ticks for the current wall-clock time (ms).
// Advances the internal pointer forward and interpolates between waypoints.
Ticks6 TrajectoryTracker::tick(double now_ms) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!active_ || waypoints_.empty()) {
        return {};
    }

    if (!initialized_) {
        start_time_ms_ = now_ms;
        initialized_   = true;
    }

    double elapsed = now_ms - start_time_ms_;

    // Advance index past waypoints that have already been reached.
    while (current_index_ + 1 < static_cast<int>(waypoints_.size())
           && waypoints_[current_index_ + 1].t_ms <= elapsed) {
        ++current_index_;
    }

    // Last waypoint — hold position and deactivate.
    if (current_index_ + 1 >= static_cast<int>(waypoints_.size())) {
        active_ = false;
        return waypoints_.back().ticks;
    }

    // Linear interpolation between current and next waypoint.
    const auto& wp0 = waypoints_[current_index_];
    const auto& wp1 = waypoints_[current_index_ + 1];
    double dt_seg = wp1.t_ms - wp0.t_ms;
    double alpha  = (dt_seg > 0.0)
                  ? std::clamp((elapsed - wp0.t_ms) / dt_seg, 0.0, 1.0)
                  : 1.0;

    Ticks6 out;
    for (int j = 0; j < NUM_JOINTS; ++j) {
        out[j] = static_cast<int16_t>(
            std::round(wp0.ticks[j] + alpha * (wp1.ticks[j] - wp0.ticks[j])));
    }
    return out;
}
