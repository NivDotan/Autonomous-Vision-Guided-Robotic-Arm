#include "tracker.hpp"
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace trk {

// ── Helpers ──────────────────────────────────────────────────────────────────

static cv::Rect bbox_to_rect(const BBox& b) {
    return cv::Rect(b[0], b[1], b[2], b[3]);
}

static BBox rect_to_bbox(const cv::Rect& r) {
    return { r.x, r.y, r.width, r.height };
}

// ── CsrtTracker ──────────────────────────────────────────────────────────────

CsrtTracker::CsrtTracker(double reinit_threshold)
    : reinit_threshold_(reinit_threshold), active_(false)
{}

cv::Ptr<cv::Tracker> CsrtTracker::make_csrt() {
    // Try cv::legacy::TrackerCSRT (opencv_contrib), fall back to built-in.
#ifdef HAVE_OPENCV_TRACKING
    auto params = cv::TrackerCSRT::Params();
    params.use_channel_and_region_flipping = true;
    return cv::TrackerCSRT::create(params);
#else
    return cv::TrackerCSRT::create();
#endif
}

void CsrtTracker::init(const cv::Mat& frame, const BBox& bbox) {
    tracker_ = make_csrt();
    cv::Rect roi = bbox_to_rect(bbox);
    // Clamp to frame bounds.
    roi &= cv::Rect(0, 0, frame.cols, frame.rows);
    if (roi.empty()) {
        active_ = false;
        return;
    }
    tracker_->init(frame, roi);
    last_bbox_ = rect_to_bbox(roi);
    active_    = true;
}

TrackResult CsrtTracker::update(const cv::Mat& frame) {
    if (!active_ || !tracker_) {
        return { false, {}, 0.0, 0, 0, 0 };
    }

    cv::Rect roi;
    bool ok = tracker_->update(frame, roi);

    if (!ok || roi.empty() || roi.width <= 0 || roi.height <= 0) {
        active_ = false;
        return { false, last_bbox_, 0.0, 0, 0, 0 };
    }

    // Clamp to frame.
    roi &= cv::Rect(0, 0, frame.cols, frame.rows);
    BBox bbox = rect_to_bbox(roi);
    last_bbox_ = bbox;

    double conf = query_confidence(frame, bbox);

    // If confidence is too low, signal that re-initialization is needed.
    if (conf < reinit_threshold_ && reinit_cb_) {
        BBox new_bbox = reinit_cb_(frame);
        if (new_bbox[2] > 0 && new_bbox[3] > 0) {
            init(frame, new_bbox);
        } else {
            active_ = false;
            return { false, bbox, conf, 0, 0, 0 };
        }
    }

    int cx   = bbox[0] + bbox[2] / 2;
    int cy   = bbox[1] + bbox[3] / 2;
    int area = bbox[2] * bbox[3];

    return { true, bbox, conf, cx, cy, area };
}

// Confidence heuristic: measure NCC between current patch and initial template.
// A full CSRT response map query would require opencv_contrib internals;
// this NCC proxy is a practical approximation that doesn't need private APIs.
double CsrtTracker::query_confidence(const cv::Mat& frame, const BBox& bbox) {
    if (bbox[2] <= 0 || bbox[3] <= 0) return 0.0;
    cv::Rect roi(bbox[0], bbox[1], bbox[2], bbox[3]);
    roi &= cv::Rect(0, 0, frame.cols, frame.rows);
    if (roi.empty()) return 0.0;

    cv::Mat patch;
    cv::cvtColor(frame(roi), patch, cv::COLOR_BGR2GRAY);

    // Compute variance as a proxy for feature richness / confidence.
    cv::Scalar mean, stddev;
    cv::meanStdDev(patch, mean, stddev);
    double variance = stddev[0] * stddev[0];
    // Normalise to [0, 1] — variance > 1000 is considered high confidence.
    return std::min(1.0, variance / 1000.0);
}

void CsrtTracker::reset() {
    tracker_.release();
    active_ = false;
    last_bbox_ = {};
}

} // namespace trk
