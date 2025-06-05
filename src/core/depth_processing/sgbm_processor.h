#ifndef SGBM_PROCESSOR_H
#define SGBM_PROCESSOR_H

#include <opencv2/core/mat.hpp>
#include <opencv2/calib3d.hpp> // For StereoSGBM
#include <string>
#include <core/common/file_utils.h> // For CameraParams struct

namespace core {
namespace depth {

struct SgbmParams {
    int minDisparity = 0;
    int numDisparities = 256; // Must be > 0 and divisible by 16
    int blockSize = 1;        // Must be odd, usually 3-11
    int P1 = 1;               // Penalty 1, P1 < P2
    int P2 = 128;             // Penalty 2
    int disp12MaxDiff = 1;
    int preFilterCap = 15;
    int uniquenessRatio = 5;  // Usually 5-15
    int speckleWindowSize = 50; // 0 to disable, or 50-200
    int speckleRange = 8;     // Usually 1-2
    int mode = cv::StereoSGBM::MODE_HH; // MODE_SGBM, MODE_HH, MODE_SGBM_3WAY, MODE_HH4

    // Helper to validate and adjust params
    void validate() {
        if (numDisparities <= 0 || numDisparities % 16 != 0) {
            numDisparities = ((numDisparities / 16) + 1) * 16; // Make it valid
            if (numDisparities <= 0) numDisparities = 16;
        }
        if (blockSize % 2 == 0) blockSize++; // Make it odd
        if (blockSize < 1) blockSize = 1;
        // Add other validations as needed
    }
};

class SgbmProcessor {
public:
    SgbmProcessor(
        const std::string& stereo_calib_filepath,
        const SgbmParams& params,
        const cv::Size& input_image_size // Expected size of input images for rectification
    );

    // Default constructor for cases where initialization might be deferred
    SgbmProcessor() = default;


    // Initialize or re-initialize the processor
    bool initialize(
        const std::string& stereo_calib_filepath,
        const SgbmParams& params,
        const cv::Size& input_image_size
    );

    // Rectifies a pair of stereo images
    bool rectify_images(
        const cv::Mat& left_raw, const cv::Mat& right_raw,
        cv::Mat& left_rectified, cv::Mat& right_rectified
    ) const;

    // Computes disparity map from rectified images
    // Assumes left_rectified and right_rectified are CV_8U grayscale
    cv::Mat compute_disparity(
        const cv::Mat& left_rectified_gray,
        const cv::Mat& right_rectified_gray
    ) const;

    // Updates SGBM parameters and recreates the matcher
    void update_sgbm_parameters(const SgbmParams& params);

    const cv::Mat& get_q_matrix() const { return Q_matrix_; }
    bool is_initialized() const { return initialized_; }
    SgbmParams get_current_sgbm_params() const { return sgbm_params_; }

private:
    void create_sgbm_matcher();

    common::CameraParams cam_params_left_;
    common::CameraParams cam_params_right_; // Contains R, T relative to left cam

    cv::Mat map1x_, map1y_; // Rectification maps for left camera
    cv::Mat map2x_, map2y_; // Rectification maps for right camera
    cv::Mat Q_matrix_;      // Perspective transformation matrix for reprojectImageTo3D

    SgbmParams sgbm_params_;
    cv::Ptr<cv::StereoSGBM> sgbm_matcher_;

    cv::Size image_size_ = {0,0}; // Size used for stereoRectify and initUndistortRectifyMap
    bool initialized_ = false;
};

} // namespace depth
} // namespace core

#endif // SGBM_PROCESSOR_H
