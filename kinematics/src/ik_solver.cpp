#define _USE_MATH_DEFINES
#include "kinematics.hpp"
#include <cmath>
#include <algorithm>

namespace kin {

// ── Helpers ──────────────────────────────────────────────────────────────────

static double clamp_angle(double q, double lo, double hi) {
    return std::max(lo, std::min(hi, q));
}

// ZYX Euler angles from rotation matrix R: R = Rz(α) Ry(β) Rx(γ)
// Returns {alpha, beta, gamma}.
static std::array<double, 3> euler_zyx(const Eigen::Matrix3d& R) {
    double beta = std::atan2(-R(2, 0), std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0)));
    double cb   = std::cos(beta);
    double alpha, gamma;
    if (std::abs(cb) < 1e-9) {
        // Gimbal lock — best effort
        alpha = 0.0;
        gamma = std::atan2(-R(1, 2), R(1, 1));
    } else {
        alpha = std::atan2(R(1, 0) / cb, R(0, 0) / cb);
        gamma = std::atan2(R(2, 1) / cb, R(2, 2) / cb);
    }
    return { alpha, beta, gamma };
}

// ── Analytical IK ────────────────────────────────────────────────────────────
//
// Spherical wrist decomposition for the SO-101 arm (Modified DH, Craig).
//
// Joints 1-3 position the wrist centre; joints 4-6 orient the EE.
// Wrist centre offset from EE along the approach axis (x in EE frame):
//   d_wrist = a5 + a_EE = 0.09 + 0.17 = 0.26 m  (along local x of joint-5 frame)
//
// The arm has:
//   L0 (base height) = d1 = 0.08 m
//   L_shoulder_col   = d2 = 0.48 m   (height of shoulder_column link)
//   L1 (upper arm)   = a2 = 0.32062 m
//   L2 (forearm)     = a3 = 0.26268 m

IKResult ik_analytical(const Mat4& T_target) {
    IKResult res;
    res.iterations = 0;
    res.residual   = 0.0;

    const Eigen::Matrix3d R_target = T_target.block<3, 3>(0, 0);
    const Vec3            p_ee     = T_target.block<3, 1>(0, 3);

    // Step 1 — wrist centre
    // EE x-axis in base frame = R_target * [1,0,0]ᵀ
    constexpr double D_WRIST = 0.09 + 0.17;  // a5 + a_EE
    Vec3 p_w = p_ee - R_target * Vec3(D_WRIST, 0.0, 0.0);

    // Step 2 — q1 (base pan)
    const double q1 = std::atan2(p_w.y(), p_w.x());

    // Step 3 — q2, q3 via planar 2R IK in the sagittal plane.
    // The shoulder_pan puts the arm in the plane; after rotating by q1 the
    // wrist centre (projected) sits at radius r and height z above the base.
    const double r  = std::sqrt(p_w.x() * p_w.x() + p_w.y() * p_w.y());
    // Height above the shoulder_lift joint (at d1 + d2 = 0.08 + 0.48 = 0.56 m)
    const double z  = p_w.z() - (g_dh[0].d + g_dh[1].d);
    const double L1 = g_dh[2].a;  // upper arm
    const double L2 = g_dh[3].a;  // forearm

    const double dist2 = r * r + z * z;
    const double D = (dist2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);

    if (std::abs(D) > 1.0) {
        res.success = false;
        return res;
    }

    // Elbow-up solution (positive square root).
    const double q3 = std::atan2(std::sqrt(1.0 - D * D), D);
    const double q2 = std::atan2(z, r)
                    - std::atan2(L2 * std::sin(q3), L1 + L2 * std::cos(q3));

    // Step 4 — wrist angles from R_{0,3}ᵀ * R_target
    // Compute R_{0,3} from q1, q2, q3.
    JointVec q_partial = { q1, q2, q3, 0.0, 0.0, 0.0 };
    Mat4 T_03 = Mat4::Identity();
    for (int i = 0; i < 3; ++i) {
        T_03 = T_03 * dh_matrix(g_dh[i], q_partial[i]);
    }
    Eigen::Matrix3d R_03 = T_03.block<3, 3>(0, 0);
    Eigen::Matrix3d R_36 = R_03.transpose() * R_target;

    // Extract q4 (wrist_flex / palm), q5 (wrist_roll), q6 (gripper) from R_36.
    // The wrist is ZYX Euler in terms of the wrist-sub-chain axes.
    // For the modified DH with α4=0, α5=π/2, α6=-π/2 the relationship is:
    //   R_36 = Rz(q4) * Ry(q5) * ... — use ZYX Euler as a practical approximation
    //   (accurate when arm doesn't operate in extreme wrist configurations).
    auto zyx = euler_zyx(R_36);
    const double q4 = zyx[0];
    const double q5 = zyx[1];
    const double q6 = zyx[2];

    res.success = true;
    res.q = { q1, q2, q3, q4, q5, q6 };
    return res;
}

// ── Damped Least Squares IK ──────────────────────────────────────────────────

IKResult ik_dls(const Mat4& T_target,
                const JointVec& q_init,
                int    max_iter,
                double lambda,
                double tol_pos) {
    IKResult res;
    res.q = q_init;

    const Vec3            p_target = T_target.block<3, 1>(0, 3);
    const Eigen::Matrix3d R_target = T_target.block<3, 3>(0, 0);

    for (int iter = 0; iter < max_iter; ++iter) {
        Mat4 T_curr = fk_transform(res.q);
        Vec3 p_curr = T_curr.block<3, 1>(0, 3);
        Eigen::Matrix3d R_curr = T_curr.block<3, 3>(0, 0);

        // Position error
        Vec3 ep = p_target - p_curr;
        double pos_err = ep.norm();

        // Orientation error (axis-angle via skew-symmetric of R_err)
        Eigen::Matrix3d R_err = R_curr.transpose() * R_target;
        Vec3 eo;
        eo << (R_err(2, 1) - R_err(1, 2)) * 0.5,
              (R_err(0, 2) - R_err(2, 0)) * 0.5,
              (R_err(1, 0) - R_err(0, 1)) * 0.5;
        eo = R_curr * eo;  // rotate to base frame

        if (pos_err < tol_pos) {
            res.success    = true;
            res.residual   = pos_err;
            res.iterations = iter;
            return res;
        }

        // Task-space error (6-vector)
        Eigen::Matrix<double, 6, 1> e;
        e.head<3>() = ep;
        e.tail<3>() = eo;

        // Damped Least Squares: Δq = Jᵀ(JJᵀ + λ²I)⁻¹ e
        auto J = geometric_jacobian(res.q);
        Eigen::Matrix<double, 6, 6> JJT = J * J.transpose();
        JJT += lambda * lambda * Eigen::Matrix<double, 6, 6>::Identity();
        Eigen::Matrix<double, 6, 1> dq = J.transpose() * JJT.ldlt().solve(e);

        for (int i = 0; i < 6; ++i) {
            res.q[i] += dq[i];
        }
    }

    res.success    = false;
    res.residual   = (p_target - fk_position(res.q)).norm();
    res.iterations = max_iter;
    return res;
}

// ── Unified solver ───────────────────────────────────────────────────────────

IKResult ik_solve(const Mat4& T_target, const JointVec& q_init) {
    IKResult res = ik_analytical(T_target);
    if (!res.success) {
        // Analytical failed (out of reach or near singularity) — try DLS.
        res = ik_dls(T_target, q_init);
    }
    return res;
}

} // namespace kin
