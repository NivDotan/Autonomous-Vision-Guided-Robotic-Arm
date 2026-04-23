#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "tracker.hpp"
#include <opencv2/core.hpp>

namespace py = pybind11;
using namespace trk;

// Convert a numpy HxWx3 uint8 array to cv::Mat (zero-copy where possible).
static cv::Mat numpy_to_mat(py::array_t<uint8_t> arr) {
    auto buf = arr.request();
    if (buf.ndim == 3) {
        return cv::Mat(buf.shape[0], buf.shape[1], CV_8UC3, buf.ptr);
    } else if (buf.ndim == 2) {
        return cv::Mat(buf.shape[0], buf.shape[1], CV_8UC1, buf.ptr);
    }
    throw std::runtime_error("numpy_to_mat: expected 2D or 3D uint8 array");
}

PYBIND11_MODULE(pytracker, m) {
    m.doc() = "C++ OpenCV CSRT tracker + Lucas-Kanade optical flow predictor.";

    py::class_<TrackResult>(m, "TrackResult")
        .def_readonly("success",    &TrackResult::success)
        .def_readonly("bbox",       &TrackResult::bbox)
        .def_readonly("confidence", &TrackResult::confidence)
        .def_readonly("cx",         &TrackResult::cx)
        .def_readonly("cy",         &TrackResult::cy)
        .def_readonly("area",       &TrackResult::area);

    py::class_<CsrtTracker>(m, "CsrtTracker")
        .def(py::init<double>(), py::arg("reinit_threshold") = 0.15,
             "Create tracker. reinit_threshold: confidence below this triggers reinit.")
        .def("init", [](CsrtTracker& t,
                        py::array_t<uint8_t> frame,
                        std::array<int, 4> bbox) {
            cv::Mat mat = numpy_to_mat(frame);
            t.init(mat, bbox);
        }, py::arg("frame"), py::arg("bbox_xywh"),
        "Initialise tracker on the given bounding box (x, y, w, h).")
        .def("update", [](CsrtTracker& t, py::array_t<uint8_t> frame) {
            cv::Mat mat = numpy_to_mat(frame);
            return t.update(mat);
        }, py::arg("frame"),
        "Track in the next frame. Returns TrackResult.")
        .def("reset",     &CsrtTracker::reset)
        .def("is_active", &CsrtTracker::is_active);

    py::class_<OpticalFlowPredictor>(m, "OpticalFlowPredictor")
        .def(py::init<>())
        .def("predict", [](OpticalFlowPredictor& p,
                           py::array_t<uint8_t> prev_gray,
                           py::array_t<uint8_t> curr_gray,
                           std::array<int, 4> bbox) {
            cv::Mat pg = numpy_to_mat(prev_gray);
            cv::Mat cg = numpy_to_mat(curr_gray);
            return p.predict(pg, cg, bbox);
        }, py::arg("prev_gray"), py::arg("curr_gray"), py::arg("bbox_xywh"),
        "Predict bbox shift using Lucas-Kanade optical flow. "
        "Both frames must be single-channel (grayscale).");
}
