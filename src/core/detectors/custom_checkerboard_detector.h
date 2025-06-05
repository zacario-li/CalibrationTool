#ifndef CUSTOM_CHECKERBOARD_DETECTOR_H
#define CUSTOM_CHECKERBOARD_DETECTOR_H

#include <opencv2/core/mat.hpp>
#include <vector>
#include <opencv2/core/types.hpp> // For cv::Point2f, cv::Size

namespace core {
namespace detectors {

// Structure to hold corner candidates with their scores
struct CornerCandidate {
    cv::Point2f point;
    float score;
    // Potentially other attributes like angle modes if needed later
    bool operator<(const CornerCandidate& other) const { // For sorting by score
        return score < other.score;
    }
};


// Main function to detect a checkerboard pattern in an image using the custom algorithm.
//
// @param gray_image Input grayscale image.
// @param board_size The number of inner corners per row and column (e.g., cv::Size(cols-1, rows-1)).
// @param corners Output vector of detected corner points (cv::Point2f).
// @param winsize Approximate window size used for various local operations (similar to Python's winsize).
// @param trim_image If true, attempts to trim the image to a region of interest first.
// @return A score indicating the quality of the detected checkerboard (e.g., Python's check_score, lower is better).
//         Returns a high score (e.g., 1.0 or higher) if detection fails or quality is poor.
//
// Note: This function aims to replicate the functionality of Python's `detect_checkerboard`.
// The Python version returns (corners_opencv, check_score). If corners_opencv is None, detection failed.
// Here, we'll return the score and fill the `corners` vector. If score is high, corners might be unreliable or empty.
double detect_checkerboard_custom(
    const cv::Mat& gray_image,
    const cv::Size& board_size, // Inner corners (cols-1, rows-1)
    std::vector<cv::Point2f>& corners,
    int winsize = 9,
    bool trim_image = false
);


// Helper function declarations (will be defined in .cpp, possibly in an anonymous namespace)
// These are internal to the detector's implementation but listed here for clarity of porting.
// Actual C++ structure might place these in an anonymous namespace in the .cpp file.

// cv::Mat normalize_image_custom(const cv::Mat& img);
// std::vector<cv::Mat> create_correlation_patch_custom(double angle1_rad, double angle2_rad, int radius);
// cv::Mat detect_corners_template_custom(const cv::Mat& gray, const std::vector<cv::Mat>& templates, const std::string& mode = "same");
// cv::Mat detect_all_corners_custom(const cv::Mat& normalized_gray, const std::vector<int>& radiuses);
// std::vector<CornerCandidate> get_corner_candidates_custom(const cv::Mat& corr_map, int step, double threshold);
// std::vector<CornerCandidate> non_maximum_suppression_custom(const std::vector<CornerCandidate>& candidates, float min_dist);
// std::vector<CornerCandidate> refine_corners_custom(const std::vector<CornerCandidate>& candidates, const cv::Mat& normalized_gray, int winsize, bool check_only = false);
// cv::Point2f solve_patch_corner_custom(const cv::Mat& patch_dx, const cv::Mat& patch_dy); // Helper for refine_corners
// double checkerboard_score_custom(const std::vector<cv::Point2f>& corners_vec, const cv::Size& current_board_size);
// bool reorder_checkerboard_custom(const std::vector<CornerCandidate>& candidates, const cv::Mat& normalized_gray, const cv::Size& target_board_size, std::vector<cv::Point2f>& ordered_corners, double& max_reorder_dist);
// cv::Mat trim_picture_custom(const cv::Mat& gray, cv::Point& crop_start_offset);


} // namespace detectors
} // namespace core

#endif // CUSTOM_CHECKERBOARD_DETECTOR_H
