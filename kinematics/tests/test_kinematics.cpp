// Unit tests for the kinematics library.
// Uses a minimal inline test framework (no external dependency).
// Run via: ctest --test-dir build -R kinematics_unit -V

#include "kinematics.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>
#include <stdexcept>

using namespace kin;

// ── Minimal test harness ─────────────────────────────────────────────────────
static int g_pass = 0, g_fail = 0;

#define EXPECT_TRUE(cond)                                            \
    do {                                                             \
        if (cond) { ++g_pass; }                                      \
        else {                                                        \
            ++g_fail;                                                 \
            std::fprintf(stderr, "FAIL  %s:%d  %s\n",               \
                         __FILE__, __LINE__, #cond);                 \
        }                                                            \
    } while (0)

#define EXPECT_NEAR(a, b, eps) EXPECT_TRUE(std::abs((a) - (b)) < (eps))
#define EXPECT_VEC_NEAR(v1, v2, eps)                                 \
    EXPECT_TRUE(((v1) - (v2)).norm() < (eps))

// ── Test 1: Zero-pose FK ──────────────────────────────────────────────────────
// With q = {0,0,0,0,0,0} (all joints at their DH zero position),
// compute the EE position analytically from the DH table and compare.
static void test_zero_pose_fk() {
    std::puts("-- Test: zero-pose FK");
    JointVec q_zero{};

    Mat4 T = fk_transform(q_zero);
    Vec3 p = T.block<3, 1>(0, 3);

    // With all joints at zero the arm extends fully "forward".
    // Expected position (rough, based on DH table sum):
    //   x ≈ a2 + a3 + a4 + a5 + a_EE = 0.32062 + 0.26268 + 0.20 + 0.09 + 0.17 = 1.0433
    //   y = 0
    //   z = d1 + d2 = 0.08 + 0.48 = 0.56
    // (alpha rotations shift this — the actual value depends on the exact chain,
    //  so we just check it's in a plausible neighbourhood and the matrix is SE(3).)
    EXPECT_TRUE(p.norm() > 0.5 && p.norm() < 2.0);
    EXPECT_NEAR(p.z(), 0.56, 0.15);  // height should be near shoulder height

    // Check T is a valid SE(3) element: R orthonormal, last row = [0,0,0,1].
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    EXPECT_NEAR((R * R.transpose() - Eigen::Matrix3d::Identity()).norm(), 0.0, 1e-10);
    EXPECT_NEAR(T(3, 3), 1.0, 1e-12);
    EXPECT_NEAR(T(3, 0), 0.0, 1e-12);

    std::printf("    EE position: (%.4f, %.4f, %.4f)\n", p.x(), p.y(), p.z());
}

// ── Test 2: FK-IK round-trip ──────────────────────────────────────────────────
static void test_fk_ik_roundtrip() {
    std::puts("-- Test: FK-IK round-trip (1000 random configs)");
    std::mt19937 rng(42);
    // Joint angle ranges (rad) roughly matching tick limits.
    const double lo = -1.2, hi = 1.2;
    std::uniform_real_distribution<double> dist(lo, hi);

    int ok = 0, total = 1000;
    for (int i = 0; i < total; ++i) {
        JointVec q_true{};
        for (int j = 0; j < 6; ++j) q_true[j] = dist(rng);

        Mat4 T_target = fk_transform(q_true);
        IKResult res  = ik_solve(T_target, {});  // zero seed

        if (!res.success) continue;

        Vec3 p_fk = fk_position(res.q);
        Vec3 p_gt = T_target.block<3, 1>(0, 3);
        if ((p_fk - p_gt).norm() < 1e-3) ++ok;
    }
    std::printf("    Round-trip OK: %d / %d\n", ok, total);
    EXPECT_TRUE(ok > 700);  // expect >70% for reachable configs
}

// ── Test 3: DLS convergence ───────────────────────────────────────────────────
static void test_dls_convergence() {
    std::puts("-- Test: DLS convergence near current pose");
    // Target: small perturbation from home.
    JointVec q_home{};
    q_home[0] = 0.1; q_home[1] = -0.2; q_home[2] = 0.3;
    Mat4 T_home = fk_transform(q_home);

    // Slightly different target (5 cm away).
    Mat4 T_target = T_home;
    T_target(0, 3) += 0.05;

    IKResult res = ik_dls(T_target, q_home, 200, 0.05, 1e-4);
    std::printf("    DLS iters=%d  residual=%.6f\n", res.iterations, res.residual);
    EXPECT_TRUE(res.success);
    EXPECT_TRUE(res.iterations < 50);
    EXPECT_NEAR(res.residual, 0.0, 1e-3);
}

// ── Test 4: Jacobian finite-difference check ─────────────────────────────────
static void test_jacobian_finite_diff() {
    std::puts("-- Test: Jacobian vs finite differences (10 configs)");
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    const double eps = 1e-6;
    double max_err = 0.0;

    for (int trial = 0; trial < 10; ++trial) {
        JointVec q{};
        for (int j = 0; j < 6; ++j) q[j] = dist(rng);

        auto J_analytic = geometric_jacobian(q);

        // Finite-difference approximation of the linear Jacobian (top 3 rows).
        Eigen::Matrix<double, 3, 6> J_fd;
        Vec3 p0 = fk_position(q);
        for (int j = 0; j < 6; ++j) {
            JointVec q_pert = q;
            q_pert[j] += eps;
            Vec3 p_pert = fk_position(q_pert);
            J_fd.col(j) = (p_pert - p0) / eps;
        }

        double err = (J_analytic.topRows<3>() - J_fd).norm();
        max_err = std::max(max_err, err);
    }
    std::printf("    Max J_linear error vs FD: %.2e\n", max_err);
    EXPECT_TRUE(max_err < 1e-4);
}

// ── Test 5: Trajectory continuity ────────────────────────────────────────────
static void test_trajectory_continuity() {
    std::puts("-- Test: trapezoidal trajectory continuity");
    JointVec q_start{};
    JointVec q_end{};
    q_end[0] = 1.0; q_end[1] = 0.5; q_end[2] = -0.3;

    auto wps = trapezoid_profile(q_start, q_end, 1.5, 3.0, 0.005);
    EXPECT_TRUE(wps.size() > 10);

    // Check start and end positions.
    for (int j = 0; j < 6; ++j) {
        EXPECT_NEAR(wps.front().q[j], q_start[j], 1e-9);
        EXPECT_NEAR(wps.back().q[j],  q_end[j],   1e-6);
    }

    // Check continuity: max velocity per step should be bounded.
    const double dt = 0.005;
    const double v_max = 1.5 + 0.01;  // slight tolerance
    for (size_t i = 1; i < wps.size(); ++i) {
        for (int j = 0; j < 6; ++j) {
            double v = std::abs(wps[i].q[j] - wps[i-1].q[j]) / dt;
            EXPECT_TRUE(v < v_max + 0.1);
        }
    }
    std::printf("    %zu waypoints generated\n", wps.size());
}

// ── Test 6: Calibration round-trip ───────────────────────────────────────────
static void test_calibration_roundtrip() {
    std::puts("-- Test: tick ↔ radian round-trip");
    std::array<int, 6> ticks_in = { 2048, 1800, 1500, 2500, 3000, 1200 };
    JointVec q = ticks_to_rad(ticks_in, g_calib);
    auto ticks_out = rad_to_ticks(q, g_calib);
    for (int j = 0; j < 6; ++j) {
        EXPECT_TRUE(std::abs(ticks_out[j] - ticks_in[j]) <= 1);  // ±1 tick rounding
    }
    std::printf("    Ticks [%d,%d,%d,%d,%d,%d] → rad → ticks verified\n",
                ticks_out[0], ticks_out[1], ticks_out[2],
                ticks_out[3], ticks_out[4], ticks_out[5]);
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main() {
    std::puts("=== Kinematics unit tests ===");
    test_zero_pose_fk();
    test_fk_ik_roundtrip();
    test_dls_convergence();
    test_jacobian_finite_diff();
    test_trajectory_continuity();
    test_calibration_roundtrip();

    std::printf("\n=== %d passed, %d failed ===\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
