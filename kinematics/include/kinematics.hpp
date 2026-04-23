#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>
#include <Eigen/Dense>

namespace kin {

// ── Fundamental types ────────────────────────────────────────────────────────
// Joint vector: [base, shoulder, elbow, palm, wrist, gripper], all in radians.
using JointVec = std::array<double, 6>;
using Mat4     = Eigen::Matrix4d;
using Vec3     = Eigen::Vector3d;

// ── Robot geometry ───────────────────────────────────────────────────────────
// One row of the Modified DH table (Craig convention):
//   T_i = Rot_x(alpha) * Trans_x(a) * Rot_z(theta + theta_offset) * Trans_z(d)
struct DHRow {
    double a;             // link length    (m)
    double alpha;         // link twist     (rad)
    double d;             // joint offset   (m)
    double theta_offset;  // constant added to joint angle (rad), usually 0
};

// Per-joint calibration for tick ↔ radian conversion.
struct JointCalib {
    double tick_offset;   // tick value at q = 0
    double sign;          // +1 or -1
    int    ticks_per_rev; // always 4096 for STS3215
};

// ── FK / Jacobian ────────────────────────────────────────────────────────────

// Single Modified-DH homogeneous transform for joint angle q.
Mat4 dh_matrix(const DHRow& row, double q);

// Forward kinematics: T_0E = T_01 * T_12 * ... * T_{up_to_joint, up_to_joint+1}
// up_to_joint=6 gives the full chain through the end-effector fixed offset.
Mat4 fk_transform(const JointVec& q, int up_to_joint = 6);

// Convenience: just the end-effector position (metres, robot base frame).
Vec3 fk_position(const JointVec& q);

// Geometric Jacobian (6×6): maps joint velocities to end-effector twist [v; ω].
Eigen::Matrix<double, 6, 6> geometric_jacobian(const JointVec& q);

// ── Inverse kinematics ───────────────────────────────────────────────────────

struct IKResult {
    bool     success;
    JointVec q;
    double   residual;    // position error (m); 0 for analytical solution
    int      iterations;  // 0 = analytical, >0 = DLS iterations used
};

// Closed-form analytical IK exploiting the spherical wrist structure.
// Returns the elbow-up solution; fails if target is out of reach.
IKResult ik_analytical(const Mat4& T_target);

// Damped Least Squares iterative IK: Δq = Jᵀ(JJᵀ + λ²I)⁻¹ e
IKResult ik_dls(const Mat4& T_target,
                const JointVec& q_init,
                int    max_iter = 200,
                double lambda   = 0.05,
                double tol_pos  = 1e-4);

// Unified solver: tries analytical first, falls back to DLS.
IKResult ik_solve(const Mat4& T_target, const JointVec& q_init);

// ── Trajectory generation ────────────────────────────────────────────────────

struct Waypoint {
    JointVec q;  // joint angles (rad)
    double   t;  // time from start (s)
};

// Natural cubic spline through q_list, uniformly timed, sampled at dt.
std::vector<Waypoint> cubic_spline(
    const std::vector<JointVec>& q_list,
    double total_time,
    double dt);

// Quintic spline with explicit boundary velocities (zero if not specified).
std::vector<Waypoint> quintic_spline(
    const std::vector<JointVec>& q_list,
    double total_time,
    double dt,
    const JointVec& qd_start = {},
    const JointVec& qd_end   = {});

// Trapezoidal velocity profile — per-joint, synchronised to slowest joint.
std::vector<Waypoint> trapezoid_profile(
    const JointVec& q_start,
    const JointVec& q_end,
    double v_max,   // rad/s
    double a_max,   // rad/s²
    double dt);

// Cartesian straight-line interpolation from T_start to T_end.
// Uses IK at each interpolated pose; q_init seeds the first IK call.
std::vector<Waypoint> cartesian_linear(
    const Mat4& T_start,
    const Mat4& T_end,
    const JointVec& q_init,
    double v_max_cart,  // m/s translational speed
    double dt);

// ── Calibration helpers ──────────────────────────────────────────────────────

// Convert raw motor ticks → joint angles in radians.
JointVec ticks_to_rad(const std::array<int, 6>& ticks,
                      const std::array<JointCalib, 6>& calib);

// Convert joint angles in radians → raw motor ticks (clamped to int).
std::array<int, 6> rad_to_ticks(const JointVec& q,
                                 const std::array<JointCalib, 6>& calib);

// Load DH table + calibration from the project's joint_sim_calibration.json.
// Must be called before any kinematics functions.
void load_calibration_json(const std::string& path);

// Global state populated by load_calibration_json.
// Row 6 (index 6) is the fixed end-effector offset (a=0.17m, all others 0).
extern std::array<DHRow, 7>          g_dh;
extern std::array<JointCalib, 6>     g_calib;

} // namespace kin
