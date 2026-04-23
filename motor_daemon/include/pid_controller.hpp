#pragma once

class PIDController {
public:
    // kp, ki, kd: proportional/integral/derivative gains
    // i_max: anti-windup clamp on the integral term (in ticks)
    // alpha: EMA coefficient for derivative filter (0 < alpha < 1; smaller = smoother)
    explicit PIDController(double kp    = 2.0,
                           double ki    = 0.1,
                           double kd    = 0.05,
                           double i_max = 500.0,
                           double alpha = 0.1);

    // Returns the control output for one timestep.
    double update(double error, double dt);

    void reset();
    void set_gains(double kp, double ki, double kd, double i_max);

    double kp() const { return kp_; }
    double ki() const { return ki_; }
    double kd() const { return kd_; }

private:
    double kp_, ki_, kd_, i_max_, alpha_;
    double integral_, prev_error_, filt_deriv_;
};
