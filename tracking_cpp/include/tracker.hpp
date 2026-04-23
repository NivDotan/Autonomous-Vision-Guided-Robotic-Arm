#pragma once
#include <array>
#include <functional>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/tracking.hpp>

namespace trk {

// Bounding box in (x, y, w, h) format, pixel coordinates.
using BBox = std::array<int, 4>;

// Result from a single tracker update call.
struct TrackResult {
    bool   success;
    BBox   bbox;          // (x, y, w, h)
    double confidence;    // 0..1 from CSRT response map (0 if unavailable)
    int    cx;            // centre x
    int    cy;            // centre y
    int    area;          // w * h
};

// ── CsrtTracker ──────────────────────────────────────────────────────────────
class CsrtTracker {
public:
    explicit CsrtTracker(double reinit_threshold = 0.15);

    // Initialise the tracker on a bounding box.
    void init(const cv::Mat& frame, const BBox& bbox);

    // Track in the next frame. Returns success and updated bbox.
    // If confidence drops below reinit_threshold, returns success=false.
    TrackResult update(const cv::Mat& frame);

    bool is_active() const { return active_; }
    void reset();

    // Register a callback invoked when confidence drops below threshold.
    // Callback receives the last valid bbox so the caller can re-initialize.
    void set_reinit_callback(std::function<BBox(const cv::Mat&)> cb) {
        reinit_cb_ = std::move(cb);
    }

private:
    cv::Ptr<cv::Tracker>    tracker_;
    BBox                    last_bbox_{};
    double                  reinit_threshold_;
    bool                    active_ = false;
    std::function<BBox(const cv::Mat&)> reinit_cb_;

    cv::Ptr<cv::Tracker> make_csrt();
    double               query_confidence(const cv::Mat& frame, const BBox& bbox);
};

// ── OpticalFlowPredictor ──────────────────────────────────────────────────────
// Lucas-Kanade sparse optical flow: predicts bbox shift between frames.
class OpticalFlowPredictor {
public:
    OpticalFlowPredictor();

    // Call with consecutive frames. Returns predicted shifted bbox.
    // The predicted bbox can seed CSRT to improve tracking on fast motion.
    BBox predict(const cv::Mat& prev_gray,
                 const cv::Mat& curr_gray,
                 const BBox& prev_bbox);

private:
    static constexpr int MAX_CORNERS = 50;
    cv::TermCriteria lk_criteria_;
};

} // namespace trk
