#define _USE_MATH_DEFINES
#include "kinematics.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace kin {

// ── Cubic spline ─────────────────────────────────────────────────────────────
// Natural cubic spline through q_list, uniformly spaced in time.
// Uses the classic tridiagonal system to compute second derivatives.

std::vector<Waypoint> cubic_spline(
    const std::vector<JointVec>& q_list,
    double total_time,
    double dt)
{
    const int n = static_cast<int>(q_list.size());
    if (n < 2) throw std::invalid_argument("cubic_spline: need at least 2 waypoints");

    const double h = total_time / (n - 1);  // uniform interval

    // For each joint independently, solve for second derivatives (M).
    // Natural BC: M[0] = M[n-1] = 0.
    std::array<std::vector<double>, 6> M;
    for (int j = 0; j < 6; ++j) {
        M[j].assign(n, 0.0);
        if (n == 2) continue;  // linear — M stays zero

        // Thomas algorithm for tridiagonal system.
        std::vector<double> rhs(n - 2);
        for (int i = 1; i < n - 1; ++i) {
            rhs[i - 1] = 6.0 / (h * h)
                * (q_list[i + 1][j] - 2.0 * q_list[i][j] + q_list[i - 1][j]);
        }

        std::vector<double> diag(n - 2, 4.0);
        std::vector<double> off(n - 3, 1.0);

        // Forward sweep
        for (int i = 1; i < n - 2; ++i) {
            double w = off[i - 1] / diag[i - 1];
            diag[i] -= w * off[i - 1];
            rhs[i]  -= w * rhs[i - 1];
        }
        // Back substitution
        std::vector<double> m(n - 2);
        m[n - 3] = rhs[n - 3] / diag[n - 3];
        for (int i = n - 4; i >= 0; --i) {
            m[i] = (rhs[i] - off[i] * m[i + 1]) / diag[i];
        }
        for (int i = 0; i < n - 2; ++i) {
            M[j][i + 1] = m[i];
        }
    }

    // Sample the spline at dt intervals.
    std::vector<Waypoint> waypoints;
    const int num_steps = static_cast<int>(std::ceil(total_time / dt)) + 1;
    waypoints.reserve(num_steps);

    for (int step = 0; step < num_steps; ++step) {
        double t_abs = std::min(static_cast<double>(step) * dt, total_time);
        // Find segment index.
        int seg = static_cast<int>(t_abs / h);
        seg = std::min(seg, n - 2);
        double t_local = t_abs - seg * h;  // time within segment [0, h]

        Waypoint wp;
        wp.t = t_abs;
        for (int j = 0; j < 6; ++j) {
            double a = q_list[seg][j];
            double b = (q_list[seg + 1][j] - q_list[seg][j]) / h
                     - h / 6.0 * (2.0 * M[j][seg] + M[j][seg + 1]);
            double c = M[j][seg] / 2.0;
            double d = (M[j][seg + 1] - M[j][seg]) / (6.0 * h);
            wp.q[j] = a + b * t_local + c * t_local * t_local + d * t_local * t_local * t_local;
        }
        waypoints.push_back(wp);
    }
    return waypoints;
}

// ── Quintic spline ────────────────────────────────────────────────────────────
// Hermite quintic through q_list with specified start/end velocities.

std::vector<Waypoint> quintic_spline(
    const std::vector<JointVec>& q_list,
    double total_time,
    double dt,
    const JointVec& qd_start,
    const JointVec& qd_end)
{
    const int n = static_cast<int>(q_list.size());
    if (n < 2) throw std::invalid_argument("quintic_spline: need at least 2 waypoints");

    // For simplicity, build a two-point quintic Hermite for the entire trajectory
    // (generalised multi-point version would require more segments).
    // Boundary: pos and vel at start and end; zero acceleration at boundaries.
    const double T = total_time;
    std::vector<Waypoint> waypoints;
    const int num_steps = static_cast<int>(std::ceil(T / dt)) + 1;
    waypoints.reserve(num_steps);

    for (int step = 0; step < num_steps; ++step) {
        double tau = std::min(static_cast<double>(step) * dt, T) / T;  // [0, 1]
        // Quintic Hermite basis polynomials.
        double h00 = 1.0 - 10 * tau*tau*tau + 15 * tau*tau*tau*tau - 6 * tau*tau*tau*tau*tau;
        double h10 = tau * (1.0 - tau) * (1.0 - tau) * (1.0 - tau) * (1.0 - tau);
        double h01 = tau*tau*tau * (10.0 - 15.0 * tau + 6.0 * tau*tau);
        double h11 = tau*tau*tau*tau * (tau - 1.0) * T;

        Waypoint wp;
        wp.t = tau * T;
        for (int j = 0; j < 6; ++j) {
            wp.q[j] = h00 * q_list[0][j]
                    + h10 * T * qd_start[j]
                    + h01 * q_list[n - 1][j]
                    + h11 * qd_end[j];
        }
        waypoints.push_back(wp);
    }
    return waypoints;
}

// ── Trapezoidal velocity profile ─────────────────────────────────────────────
// Per-joint trapezoidal profiles synchronised so all joints finish together.

std::vector<Waypoint> trapezoid_profile(
    const JointVec& q_start,
    const JointVec& q_end,
    double v_max,
    double a_max,
    double dt)
{
    if (v_max <= 0.0 || a_max <= 0.0) {
        throw std::invalid_argument("trapezoid_profile: v_max and a_max must be positive");
    }

    // Compute duration needed for each joint independently.
    double total_time = 0.0;
    for (int j = 0; j < 6; ++j) {
        double dist = std::abs(q_end[j] - q_start[j]);
        // Time to reach v_max and decelerate back to 0.
        double t_ramp = v_max / a_max;
        double d_ramp  = 0.5 * a_max * t_ramp * t_ramp;
        double t_j;
        if (dist <= 2.0 * d_ramp) {
            // Triangular profile — v_max is never reached.
            t_j = 2.0 * std::sqrt(dist / a_max);
        } else {
            t_j = 2.0 * t_ramp + (dist - 2.0 * d_ramp) / v_max;
        }
        total_time = std::max(total_time, t_j);
    }

    if (total_time < dt) {
        // Already at target.
        return { { q_end, 0.0 } };
    }

    // Re-scale each joint's v_max to fill total_time (time-synchronisation).
    std::vector<Waypoint> waypoints;
    const int num_steps = static_cast<int>(std::ceil(total_time / dt)) + 1;
    waypoints.reserve(num_steps);

    // Per-joint scaled parameters.
    std::array<double, 6> sign_j{}, dist_j{}, v_j{}, a_j{}, t_ramp_j{}, d_ramp_j{};
    for (int j = 0; j < 6; ++j) {
        dist_j[j]   = std::abs(q_end[j] - q_start[j]);
        sign_j[j]   = (q_end[j] >= q_start[j]) ? 1.0 : -1.0;
        if (dist_j[j] < 1e-10) { v_j[j] = a_j[j] = 0.0; continue; }
        // Solve for v_scaled so that trapezoidal motion takes total_time.
        // total_time = 2*(v_scaled/a_scaled) + (dist - v_scaled²/a_scaled)/v_scaled
        //            (assume a_scaled = a_max, solve for v_scaled)
        double discriminant = a_max * (a_max * total_time * total_time - 4.0 * dist_j[j]);
        if (discriminant < 0.0) {
            // Must be triangular at higher acceleration.
            a_j[j]      = 4.0 * dist_j[j] / (total_time * total_time);
            v_j[j]      = a_j[j] * total_time / 2.0;
        } else {
            a_j[j]      = a_max;
            v_j[j]      = (a_max * total_time - std::sqrt(discriminant)) / 2.0;
        }
        t_ramp_j[j] = v_j[j] / a_j[j];
        d_ramp_j[j] = 0.5 * a_j[j] * t_ramp_j[j] * t_ramp_j[j];
    }

    for (int step = 0; step < num_steps; ++step) {
        double t = std::min(static_cast<double>(step) * dt, total_time);
        Waypoint wp;
        wp.t = t;
        for (int j = 0; j < 6; ++j) {
            if (dist_j[j] < 1e-10) { wp.q[j] = q_start[j]; continue; }
            double pos;
            if (t <= t_ramp_j[j]) {
                // Acceleration phase.
                pos = 0.5 * a_j[j] * t * t;
            } else if (t <= total_time - t_ramp_j[j]) {
                // Constant velocity.
                pos = d_ramp_j[j] + v_j[j] * (t - t_ramp_j[j]);
            } else {
                // Deceleration phase.
                double t2 = total_time - t;
                pos = dist_j[j] - 0.5 * a_j[j] * t2 * t2;
            }
            wp.q[j] = q_start[j] + sign_j[j] * std::min(pos, dist_j[j]);
        }
        waypoints.push_back(wp);
    }
    return waypoints;
}

// ── Cartesian linear interpolation ───────────────────────────────────────────
// Straight-line in task space, IK at each sampled pose.

std::vector<Waypoint> cartesian_linear(
    const Mat4& T_start,
    const Mat4& T_end,
    const JointVec& q_init,
    double v_max_cart,
    double dt)
{
    Vec3 p_start = T_start.block<3, 1>(0, 3);
    Vec3 p_end   = T_end.block<3, 1>(0, 3);
    double dist  = (p_end - p_start).norm();

    if (dist < 1e-6) return { { q_init, 0.0 } };

    double total_time = dist / v_max_cart;
    int num_steps = static_cast<int>(std::ceil(total_time / dt)) + 1;

    // Slerp for rotation.
    Eigen::Quaterniond q_start_rot(T_start.block<3, 3>(0, 0));
    Eigen::Quaterniond q_end_rot(T_end.block<3, 3>(0, 0));

    std::vector<Waypoint> waypoints;
    waypoints.reserve(num_steps);

    JointVec q_prev = q_init;
    for (int step = 0; step < num_steps; ++step) {
        double tau = std::min(static_cast<double>(step) * dt / total_time, 1.0);
        Vec3 p_interp = (1.0 - tau) * p_start + tau * p_end;
        Eigen::Quaterniond q_interp = q_start_rot.slerp(tau, q_end_rot);

        Mat4 T_interp = Mat4::Identity();
        T_interp.block<3, 3>(0, 0) = q_interp.toRotationMatrix();
        T_interp.block<3, 1>(0, 3) = p_interp;

        IKResult res = ik_solve(T_interp, q_prev);
        Waypoint wp;
        wp.t = static_cast<double>(step) * dt;
        wp.q = res.success ? res.q : q_prev;
        waypoints.push_back(wp);
        q_prev = wp.q;
    }
    return waypoints;
}

} // namespace kin
