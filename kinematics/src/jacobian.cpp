#define _USE_MATH_DEFINES
#include "kinematics.hpp"
#include <cmath>

namespace kin {

// Geometric Jacobian (6×6).
//
// For a revolute joint i with z-axis z_i (column 2 of R_{0,i}) and origin o_i:
//   Linear  component:  z_i × (o_EE - o_i)
//   Angular component:  z_i
//
// Returns J such that [v; ω] = J * dq/dt.
Eigen::Matrix<double, 6, 6> geometric_jacobian(const JointVec& q) {
    // Pre-compute transforms T_{0,0}, T_{0,1}, ..., T_{0,6}.
    std::array<Mat4, 7> T;
    T[0] = Mat4::Identity();
    for (int i = 0; i < 6; ++i) {
        T[i + 1] = T[i] * dh_matrix(g_dh[i], q[i]);
    }
    // End-effector position (including fixed EE offset).
    Mat4 T_EE = T[6] * dh_matrix(g_dh[6], 0.0);
    Vec3 o_EE = T_EE.block<3, 1>(0, 3);

    Eigen::Matrix<double, 6, 6> J;
    for (int i = 0; i < 6; ++i) {
        // z-axis of frame i: third column of R_{0,i}.
        Vec3 z_i = T[i].block<3, 1>(0, 2);
        // Origin of frame i.
        Vec3 o_i = T[i].block<3, 1>(0, 3);

        J.block<3, 1>(0, i) = z_i.cross(o_EE - o_i);  // linear
        J.block<3, 1>(3, i) = z_i;                      // angular
    }
    return J;
}

} // namespace kin
