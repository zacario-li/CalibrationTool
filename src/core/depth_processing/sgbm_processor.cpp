#include "sgbm_processor.h"
#include <opencv2/imgproc.hpp>
#include <iostream> // For std::cerr

namespace core {
namespace depth {

SgbmProcessor::SgbmProcessor(
    const std::string& stereo_calib_filepath,
    const SgbmParams& params,
    const cv::Size& input_image_size) {
    initialize(stereo_calib_filepath, params, input_image_size);
}

bool SgbmProcessor::initialize(
    const std::string& stereo_calib_filepath,
    const SgbmParams& params,
    const cv::Size& input_image_size) {

    initialized_ = false;
    image_size_ = input_image_size; // Store the target image size for rectification maps

    // Load camera parameters
    // Left camera parameters (CameraParameters1 or CameraParameters)
    cam_params_left_ = common::load_camera_param_from_json(stereo_calib_filepath, false, 0, false, true);
    if (cam_params_left_.intrinsic_matrix.empty() || cam_params_left_.distortion_coeffs.empty()) {
        std::cerr << "SgbmProcessor Error: Failed to load left camera parameters from " << stereo_calib_filepath << std::endl;
        return false;
    }
     // If image size wasn't in JSON, use provided input_image_size. Otherwise, JSON's image_size takes precedence.
    if (cam_params_left_.is_image_size_loaded && cam_params_left_.image_size != input_image_size) {
        std::cout << "SgbmProcessor Info: Image size from JSON (" << cam_params_left_.image_size
                  << ") overrides provided input_image_size (" << input_image_size << ")." << std::endl;
        image_size_ = cam_params_left_.image_size;
    } else if (!cam_params_left_.is_image_size_loaded && input_image_size.empty()) {
         std::cerr << "SgbmProcessor Error: Image size not provided and not found in JSON." << std::endl;
        return false;
    } else if (!cam_params_left_.is_image_size_loaded) {
        // Use provided input_image_size if not in JSON
        cam_params_left_.image_size = input_image_size;
    }


    // Right camera parameters (CameraParameters2) including R and T relative to camera 1
    cam_params_right_ = common::load_camera_param_from_json(stereo_calib_filepath, true, 1, false, false); // true for stereo extrinsics
    if (cam_params_right_.intrinsic_matrix.empty() || cam_params_right_.distortion_coeffs.empty() ||
        cam_params_right_.rotation_matrix_cam2.empty() || cam_params_right_.translation_vector_cam2.empty()) {
        std::cerr << "SgbmProcessor Error: Failed to load right camera parameters or stereo extrinsics from " << stereo_calib_filepath << std::endl;
        return false;
    }

    // Perform stereo rectification
    cv::Mat R1, R2, P1, P2; // Output rectification transforms and projection matrices
    cv::stereoRectify(
        cam_params_left_.intrinsic_matrix, cam_params_left_.distortion_coeffs,
        cam_params_right_.intrinsic_matrix, cam_params_right_.distortion_coeffs,
        image_size_, // Use the determined image size
        cam_params_right_.rotation_matrix_cam2, cam_params_right_.translation_vector_cam2,
        R1, R2, P1, P2, Q_matrix_ // Q_matrix_ is a member
        // flags: CALIB_ZERO_DISPARITY often used, alpha: -1 for default (all pixels retained), 0 for valid pixels only
        // For SGBM, it's common to use alpha=0 to ensure only valid regions are processed or to handle ROI carefully.
        // Python code doesn't specify flags or alpha, so defaults are used by OpenCV.
        // Let's use alpha = -1 (default) to match Python's apparent behavior.
    );

    // Compute rectification maps
    cv::initUndistortRectifyMap(
        cam_params_left_.intrinsic_matrix, cam_params_left_.distortion_coeffs, R1, P1,
        image_size_, CV_32FC1, map1x_, map1y_
    );
    cv::initUndistortRectifyMap(
        cam_params_right_.intrinsic_matrix, cam_params_right_.distortion_coeffs, R2, P2,
        image_size_, CV_32FC1, map2x_, map2y_
    );

    // Initialize SGBM matcher
    sgbm_params_ = params;
    sgbm_params_.validate(); // Ensure params are valid (e.g. numDisparities % 16 == 0)
    create_sgbm_matcher();

    initialized_ = true;
    return true;
}

void SgbmProcessor::create_sgbm_matcher() {
    sgbm_matcher_ = cv::StereoSGBM::create(
        sgbm_params_.minDisparity,
        sgbm_params_.numDisparities,
        sgbm_params_.blockSize,
        sgbm_params_.P1,
        sgbm_params_.P2,
        sgbm_params_.disp12MaxDiff,
        sgbm_params_.preFilterCap,
        sgbm_params_.uniquenessRatio,
        sgbm_params_.speckleWindowSize,
        sgbm_params_.speckleRange,
        sgbm_params_.mode
    );
}

void SgbmProcessor::update_sgbm_parameters(const SgbmParams& params) {
    if (!initialized_) {
        std::cerr << "SgbmProcessor Warning: Trying to update params on uninitialized processor." << std::endl;
        // Optionally, could try to initialize if other necessary params were stored from a failed init.
        return;
    }
    sgbm_params_ = params;
    sgbm_params_.validate();
    create_sgbm_matcher(); // Recreate matcher with new parameters
}

bool SgbmProcessor::rectify_images(
    const cv::Mat& left_raw, const cv::Mat& right_raw,
    cv::Mat& left_rectified, cv::Mat& right_rectified) const {
    if (!initialized_) {
        std::cerr << "SgbmProcessor Error: Processor not initialized for rectify_images." << std::endl;
        return false;
    }
    if (left_raw.empty() || right_raw.empty()) {
        std::cerr << "SgbmProcessor Error: Input images for rectification are empty." << std::endl;
        return false;
    }
    // Ensure input images match the size used for initUndistortRectifyMap if maps are not scaled.
    // Or, resize maps if input image size changes (more complex). For now, assume consistent size.
    if (left_raw.size() != image_size_ || right_raw.size() != image_size_) {
         std::cerr << "SgbmProcessor Warning: Input image size (" << left_raw.size()
                   << ") differs from initialization size (" << image_size_
                   << "). Rectification might be incorrect if maps are not scaled." << std::endl;
        // One could resize images here, or recompute maps. For now, proceed.
    }


    cv::remap(left_raw, left_rectified, map1x_, map1y_, cv::INTER_LINEAR);
    cv::remap(right_raw, right_rectified, map2x_, map2y_, cv::INTER_LINEAR);
    return true;
}

cv::Mat SgbmProcessor::compute_disparity(
    const cv::Mat& left_rectified_gray,
    const cv::Mat& right_rectified_gray) const {
    if (!initialized_ || !sgbm_matcher_) {
        std::cerr << "SgbmProcessor Error: Processor or SGBM matcher not initialized for compute_disparity." << std::endl;
        return cv::Mat();
    }
    if (left_rectified_gray.empty() || right_rectified_gray.empty()) {
        std::cerr << "SgbmProcessor Error: Input rectified images for disparity computation are empty." << std::endl;
        return cv::Mat();
    }
    if (left_rectified_gray.type() != CV_8U || right_rectified_gray.type() != CV_8U) {
        std::cerr << "SgbmProcessor Error: Input rectified images must be 8-bit grayscale for SGBM." << std::endl;
        // Could attempt conversion here, but better to enforce it by caller.
        return cv::Mat();
    }

    cv::Mat disparity_map;
    sgbm_matcher_->compute(left_rectified_gray, right_rectified_gray, disparity_map);

    // Disparity map from SGBM is typically 16-bit signed (CV_16S), values are 16 * disparity.
    // It might need conversion for visualization (e.g., to CV_8U after normalization).
    return disparity_map;
}

} // namespace depth
} // namespace core
