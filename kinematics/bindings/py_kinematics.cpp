#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "kinematics.hpp"

namespace py = pybind11;
using namespace kin;

PYBIND11_MODULE(pykinematics, m) {
    m.doc() = "C++ kinematics library for the SO-101 6-DOF robot arm.\n"
              "Implements FK, analytical IK, DLS IK, geometric Jacobian,\n"
              "and trajectory generation (cubic spline, trapezoid, Cartesian).";

    // ── Data types ──────────────────────────────────────────────────────────
    py::class_<DHRow>(m, "DHRow")
        .def(py::init<double, double, double, double>(),
             py::arg("a"), py::arg("alpha"), py::arg("d"), py::arg("theta_offset") = 0.0)
        .def_readwrite("a",            &DHRow::a)
        .def_readwrite("alpha",        &DHRow::alpha)
        .def_readwrite("d",            &DHRow::d)
        .def_readwrite("theta_offset", &DHRow::theta_offset)
        .def("__repr__", [](const DHRow& r) {
            return "DHRow(a=" + std::to_string(r.a)
                 + ", alpha=" + std::to_string(r.alpha)
                 + ", d=" + std::to_string(r.d) + ")";
        });

    py::class_<JointCalib>(m, "JointCalib")
        .def(py::init<double, double, int>(),
             py::arg("tick_offset"), py::arg("sign") = 1.0, py::arg("ticks_per_rev") = 4096)
        .def_readwrite("tick_offset",   &JointCalib::tick_offset)
        .def_readwrite("sign",          &JointCalib::sign)
        .def_readwrite("ticks_per_rev", &JointCalib::ticks_per_rev);

    py::class_<IKResult>(m, "IKResult")
        .def_readonly("success",    &IKResult::success)
        .def_readonly("q",          &IKResult::q)
        .def_readonly("residual",   &IKResult::residual)
        .def_readonly("iterations", &IKResult::iterations)
        .def("__repr__", [](const IKResult& r) {
            return std::string("IKResult(success=")
                 + (r.success ? "True" : "False")
                 + ", residual=" + std::to_string(r.residual)
                 + ", iters=" + std::to_string(r.iterations) + ")";
        });

    py::class_<Waypoint>(m, "Waypoint")
        .def_readonly("q", &Waypoint::q)
        .def_readonly("t", &Waypoint::t)
        .def("__repr__", [](const Waypoint& w) {
            return "Waypoint(t=" + std::to_string(w.t) + ")";
        });

    // ── Configuration ───────────────────────────────────────────────────────
    m.def("load_calibration_json", &load_calibration_json,
          py::arg("path"),
          "Load DH + calibration from the project's joint_sim_calibration.json.");

    // ── Forward kinematics ──────────────────────────────────────────────────
    m.def("dh_matrix", &dh_matrix,
          py::arg("row"), py::arg("q"),
          "4×4 Modified DH transform for one joint.");

    m.def("fk_transform", &fk_transform,
          py::arg("q"), py::arg("up_to_joint") = 6,
          "Full FK chain T_0E as 4×4 numpy array. Returns Eigen::Matrix4d.");

    m.def("fk_position", &fk_position,
          py::arg("q"),
          "End-effector position (x, y, z) in metres, robot base frame.");

    // ── Jacobian ────────────────────────────────────────────────────────────
    m.def("geometric_jacobian", &geometric_jacobian,
          py::arg("q"),
          "6×6 geometric Jacobian mapping joint velocities → EE twist [v; ω].");

    // ── Inverse kinematics ──────────────────────────────────────────────────
    m.def("ik_analytical", &ik_analytical,
          py::arg("T_target"),
          "Closed-form IK (spherical wrist decomposition). Elbow-up solution.");

    m.def("ik_dls", &ik_dls,
          py::arg("T_target"), py::arg("q_init"),
          py::arg("max_iter") = 200,
          py::arg("lambda")   = 0.05,
          py::arg("tol_pos")  = 1e-4,
          "Damped Least Squares iterative IK.");

    m.def("ik_solve", &ik_solve,
          py::arg("T_target"), py::arg("q_init"),
          "Unified IK: tries analytical first, falls back to DLS.");

    // ── Trajectory ──────────────────────────────────────────────────────────
    m.def("cubic_spline", &cubic_spline,
          py::arg("q_list"), py::arg("total_time"), py::arg("dt"),
          "Natural cubic spline through q_list, sampled at dt intervals.");

    m.def("quintic_spline", &quintic_spline,
          py::arg("q_list"), py::arg("total_time"), py::arg("dt"),
          py::arg("qd_start") = JointVec{},
          py::arg("qd_end")   = JointVec{},
          "Quintic Hermite spline with optional boundary velocities.");

    m.def("trapezoid_profile", &trapezoid_profile,
          py::arg("q_start"), py::arg("q_end"),
          py::arg("v_max"), py::arg("a_max"), py::arg("dt"),
          "Time-synchronised trapezoidal velocity profile.");

    m.def("cartesian_linear", &cartesian_linear,
          py::arg("T_start"), py::arg("T_end"), py::arg("q_init"),
          py::arg("v_max_cart"), py::arg("dt"),
          "Cartesian straight-line interpolation with IK at each sample.");

    // ── Calibration ─────────────────────────────────────────────────────────
    m.def("ticks_to_rad", [](const std::array<int, 6>& ticks,
                              const std::array<JointCalib, 6>& calib) {
        return ticks_to_rad(ticks, calib);
    }, py::arg("ticks"), py::arg("calib"));

    m.def("rad_to_ticks", [](const JointVec& q,
                              const std::array<JointCalib, 6>& calib) {
        return rad_to_ticks(q, calib);
    }, py::arg("q"), py::arg("calib"));

    // Convenience: use global calibration loaded from JSON.
    m.def("ticks_to_rad_global", [](const std::array<int, 6>& ticks) {
        return ticks_to_rad(ticks, g_calib);
    }, py::arg("ticks"),
    "Convert ticks → radians using the globally loaded calibration.");

    m.def("rad_to_ticks_global", [](const JointVec& q) {
        return rad_to_ticks(q, g_calib);
    }, py::arg("q"),
    "Convert radians → ticks using the globally loaded calibration.");
}
