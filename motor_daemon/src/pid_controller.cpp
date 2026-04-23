#include "pid_controller.hpp"
#include <algorithm>
#include <cmath>

// ── PIDController ─────────────────────────────────────────────────────────────

PIDController::PIDController(double kp, double ki, double kd,
                             double i_max, double alpha)
    : kp_(kp), ki_(ki), kd_(kd), i_max_(i_max), alpha_(alpha),
      integral_(0.0), prev_error_(0.0), filt_deriv_(0.0)
{}

double PIDController::update(double error, double dt) {
    // Integral with anti-windup clamp.
    integral_ = std::clamp(integral_ + error * dt, -i_max_, i_max_);

    // Derivative with EMA low-pass filter (reduces noise amplification).
    double raw_deriv = (error - prev_error_) / (dt > 0.0 ? dt : 1e-6);
    filt_deriv_ = alpha_ * raw_deriv + (1.0 - alpha_) * filt_deriv_;
    prev_error_ = error;

    return kp_ * error + ki_ * integral_ + kd_ * filt_deriv_;
}

void PIDController::reset() {
    integral_   = 0.0;
    prev_error_ = 0.0;
    filt_deriv_ = 0.0;
}

void PIDController::set_gains(double kp, double ki, double kd, double i_max) {
    kp_    = kp;
    ki_    = ki;
    kd_    = kd;
    i_max_ = i_max;
    reset();
}
