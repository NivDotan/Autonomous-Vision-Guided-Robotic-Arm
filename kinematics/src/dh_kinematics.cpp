#define _USE_MATH_DEFINES
#include "kinematics.hpp"
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <nlohmann/json.hpp>

namespace kin {

// ── Global state ─────────────────────────────────────────────────────────────
// Default DH table derived from so101_simple_sim.urdf + joint_sim_calibration.json.
// Modified DH (Craig): T_i = Rot_x(α) * Trans_x(a) * Rot_z(θ) * Trans_z(d)
//
// Link lengths:
//   elbow  a2 = sqrt(0.02² + 0.32²) ≈ 0.32062 m
//   palm   a3 = sqrt(0.08² + 0.25²) ≈ 0.26268 m
//
// Row indices 0..5 = joints 1..6; row 6 = fixed EE offset.
std::array<DHRow, 7> g_dh = {{
    // a,       alpha,          d,      theta_offset
    { 0.0,      0.0,            0.08,   0.0 },  // 0: base      (shoulder_pan,  Z-axis)
    { 0.0,      M_PI / 2.0,     0.48,   0.0 },  // 1: shoulder  (shoulder_lift, Y-axis)
    { 0.32062,  0.0,            0.0,    0.0 },  // 2: elbow     (elbow_flex,    Y-axis)
    { 0.26268,  0.0,            0.0,    0.0 },  // 3: palm      (wrist_flex,    Y-axis)
    { 0.20,     M_PI / 2.0,     0.0,    0.0 },  // 4: wrist     (wrist_roll,    X-axis)
    { 0.09,    -M_PI / 2.0,     0.0,    0.0 },  // 5: gripper   (gripper,       Z-axis)
    { 0.17,     0.0,            0.0,    0.0 },  // 6: EE fixed offset (gripper_tip)
}};

// Default calibration (offsets from joint_sim_calibration.json, sign=+1, tpr=4096).
std::array<JointCalib, 6> g_calib = {{
    { 2365.0, 1.0, 4096 },  // base
    { 1740.0, 1.0, 4096 },  // shoulder
    { 1410.0, 1.0, 4096 },  // elbow
    { 3000.0, 1.0, 4096 },  // palm
    { 3200.0, 1.0, 4096 },  // wrist
    { 3000.0, 1.0, 4096 },  // gripper
}};

// ── Single DH transform ──────────────────────────────────────────────────────

Mat4 dh_matrix(const DHRow& row, double q) {
    const double theta = q + row.theta_offset;
    const double ct = std::cos(theta);
    const double st = std::sin(theta);
    const double ca = std::cos(row.alpha);
    const double sa = std::sin(row.alpha);

    // Modified DH (Craig):
    // [ cos(θ)        -sin(θ)         0        a         ]
    // [ sin(θ)cos(α)   cos(θ)cos(α)  -sin(α)  -sin(α)*d ]
    // [ sin(θ)sin(α)   cos(θ)sin(α)   cos(α)   cos(α)*d ]
    // [ 0              0              0        1          ]
    Mat4 T;
    T << ct,      -st,       0.0,    row.a,
         st * ca,  ct * ca, -sa,    -sa * row.d,
         st * sa,  ct * sa,  ca,     ca * row.d,
         0.0,      0.0,      0.0,    1.0;
    return T;
}

// ── Forward kinematics ───────────────────────────────────────────────────────

Mat4 fk_transform(const JointVec& q, int up_to_joint) {
    Mat4 T = Mat4::Identity();
    // Joints 0..(up_to_joint-1) from the DH table.
    for (int i = 0; i < up_to_joint && i < 6; ++i) {
        T = T * dh_matrix(g_dh[i], q[i]);
    }
    // If computing the full chain (up_to_joint == 6), append the fixed EE offset.
    if (up_to_joint >= 6) {
        T = T * dh_matrix(g_dh[6], 0.0);
    }
    return T;
}

Vec3 fk_position(const JointVec& q) {
    return fk_transform(q).block<3, 1>(0, 3);
}

// ── Calibration JSON loader ──────────────────────────────────────────────────

void load_calibration_json(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("load_calibration_json: cannot open " + path);
    }
    nlohmann::json j;
    f >> j;

    // Map logical joint names to g_calib indices.
    const std::array<std::string, 6> order = {
        "base", "shoulder", "elbow", "palm", "wrist", "gripper"
    };

    for (const auto& entry : j.at("joints")) {
        const std::string logical = entry.at("logical").get<std::string>();
        for (int i = 0; i < 6; ++i) {
            if (order[i] == logical) {
                g_calib[i].tick_offset  = entry.at("tick_offset").get<double>();
                g_calib[i].sign         = entry.value("sign", 1.0);
                g_calib[i].ticks_per_rev = entry.value("ticks_per_rev", 4096);
                break;
            }
        }
    }
}

// ── Calibration helpers ──────────────────────────────────────────────────────

JointVec ticks_to_rad(const std::array<int, 6>& ticks,
                      const std::array<JointCalib, 6>& calib) {
    JointVec q{};
    for (int i = 0; i < 6; ++i) {
        q[i] = calib[i].sign
             * (ticks[i] - calib[i].tick_offset)
             * (2.0 * M_PI / calib[i].ticks_per_rev);
    }
    return q;
}

std::array<int, 6> rad_to_ticks(const JointVec& q,
                                 const std::array<JointCalib, 6>& calib) {
    std::array<int, 6> ticks{};
    for (int i = 0; i < 6; ++i) {
        double raw = q[i] / calib[i].sign / (2.0 * M_PI / calib[i].ticks_per_rev)
                   + calib[i].tick_offset;
        ticks[i] = static_cast<int>(std::round(raw));
    }
    return ticks;
}

} // namespace kin
