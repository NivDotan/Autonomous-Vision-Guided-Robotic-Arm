#include "tracker.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <algorithm>

namespace trk {

OpticalFlowPredictor::OpticalFlowPredictor()
    : lk_criteria_(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03)
{}

BBox OpticalFlowPredictor::predict(
    const cv::Mat& prev_gray,
    const cv::Mat& curr_gray,
    const BBox& prev_bbox)
{
    if (prev_gray.empty() || curr_gray.empty()) return prev_bbox;
    if (prev_bbox[2] <= 0 || prev_bbox[3] <= 0) return prev_bbox;

    // Detect good features to track inside the previous bounding box.
    cv::Rect roi(prev_bbox[0], prev_bbox[1], prev_bbox[2], prev_bbox[3]);
    roi &= cv::Rect(0, 0, prev_gray.cols, prev_gray.rows);
    if (roi.empty()) return prev_bbox;

    cv::Mat patch_prev = prev_gray(roi);
    std::vector<cv::Point2f> prev_pts;
    cv::goodFeaturesToTrack(patch_prev, prev_pts, MAX_CORNERS, 0.01, 3.0);

    if (prev_pts.empty()) return prev_bbox;

    // Shift points to full-frame coordinates.
    for (auto& p : prev_pts) {
        p.x += roi.x;
        p.y += roi.y;
    }

    // Lucas-Kanade optical flow.
    std::vector<cv::Point2f> curr_pts;
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prev_gray, curr_gray,
                             prev_pts, curr_pts,
                             status, err,
                             cv::Size(21, 21), 3, lk_criteria_);

    // Compute mean displacement from successfully tracked points.
    float dx = 0.0f, dy = 0.0f;
    int n_good = 0;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            dx += curr_pts[i].x - prev_pts[i].x;
            dy += curr_pts[i].y - prev_pts[i].y;
            ++n_good;
        }
    }

    if (n_good == 0) return prev_bbox;

    dx /= n_good;
    dy /= n_good;

    return {
        static_cast<int>(prev_bbox[0] + std::round(dx)),
        static_cast<int>(prev_bbox[1] + std::round(dy)),
        prev_bbox[2],
        prev_bbox[3]
    };
}

} // namespace trk
