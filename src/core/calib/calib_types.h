#ifndef CALIB_TYPES_H
#define CALIB_TYPES_H

#include <opencv2/core/mat.hpp>
#include <vector>
#include <string>
#include <core/common/app_errors.h> // For CalibErrType

namespace core {
namespace calib {

// Enum for calibration pattern types
enum class CalibPatternType {
    CHESSBOARD,
    CHARUCO,
    APRILTAG, // Not yet implemented in detail, but good to have
    UNKNOWN
};

// Result of processing a single image for corner detection and pose
struct ImagePoseResult {
    std::string image_path;
    bool corners_found = false;
    std::vector<cv::Point2f> image_points; // Detected 2D corners
    cv::Mat rvec;                          // Rotation vector (camera to pattern)
    cv::Mat tvec;                          // Translation vector (camera to pattern)
    double reprojection_error = -1.0;      // For this specific image, if applicable
    core::common::CalibErrType error_type = core::common::CalibErrType::CAL_OK;
};

// Overall result for monocular camera calibration
struct MonoCalibResult {
    core::common::CalibErrType status = core::common::CalibErrType::CAL_OK;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    std::vector<cv::Mat> rvecs; // Per-image rotation vectors
    std::vector<cv::Mat> tvecs; // Per-image translation vectors
    double overall_reprojection_error = -1.0;
    cv::Size image_size;
    std::vector<std::string> successfully_calibrated_images;
    std::vector<std::string> rejected_images;
    std::vector<std::vector<cv::Point2f>> all_image_points; // All successfully detected image points
    std::vector<std::vector<cv::Point3f>> all_object_points; // Corresponding object points for successful images
};

// Overall result for stereo camera calibration
struct StereoCalibResult {
    core::common::CalibErrType status = core::common::CalibErrType::CAL_OK;
    cv::Mat camera_matrix1;
    cv::Mat dist_coeffs1;
    cv::Mat camera_matrix2;
    cv::Mat dist_coeffs2;
    cv::Mat R; // Rotation matrix between cameras
    cv::Mat T; // Translation vector between cameras
    cv::Mat E; // Essential matrix
    cv::Mat F; // Fundamental matrix
    double overall_reprojection_error = -1.0;
    cv::Size image_size;

    // Optional: Per-image details if needed
    std::vector<std::string> successfully_calibrated_image_pairs_left;
    std::vector<std::string> successfully_calibrated_image_pairs_right;
    std::vector<std::string> rejected_image_pairs;
};

// Helper struct for asynchronous corner finding results
struct ImageCornersResult {
    std::string image_path;
    bool found;
    std::vector<cv::Point2f> corners;
    std::vector<cv::Point3f> object_points_for_image; // Object points corresponding to this image's corners
                                                      // (can be the same obj_p for all if pattern is static)
    core::common::CalibErrType error_code = core::common::CalibErrType::CAL_OK;
};

// Result for Hand-Eye calibration (AX=XB or AX=ZB)
struct HandEyeResult {
    core::common::CalibErrType status = core::common::CalibErrType::CAL_OK;
    cv::Mat X; // The transformation matrix (e.g., camera_H_gripper for AX=XB)
               // For AX=ZB, this would be world_H_base or similar.
    double rotation_error = -1.0;    // In degrees, specific to validation method
    double translation_error = -1.0; // In mm (or units of input), specific to validation method
};


} // namespace calib
} // namespace core

#endif // CALIB_TYPES_H
