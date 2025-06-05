#ifndef FILE_UTILS_H
#define FILE_UTILS_H

#include <string>
#include <vector>
#include <opencv2/core/mat.hpp>

namespace core {
namespace common {

struct CameraParams {
    cv::Mat intrinsic_matrix;
    cv::Mat distortion_coeffs;
    cv::Mat rotation_matrix_cam2;    // For stereo: rotation of camera 2 relative to camera 1
    cv::Mat translation_vector_cam2; // For stereo: translation of camera 2 relative to camera 1
    cv::Size image_size;
    bool is_stereo_param_loaded = false; // Flag to indicate if R and T for cam2 were loaded
    bool is_image_size_loaded = false; // Flag to indicate if image size was loaded
};

// Loads camera intrinsic and distortion parameters from a JSON file.
// Optionally loads stereo extrinsics (R, T for camera 2) and image size if requested.
CameraParams load_camera_param_from_json(
    const std::string& filepath,
    bool load_stereo_extrinsics = false,
    int camera_id_for_stereo = 0, // 0 for CameraParameters1, 1 for CameraParameters2
    bool transpose_intrinsic = false,
    bool load_image_size = false
);

// Loads Hand-Eye calibration matrix (e.g., AXXB or AXZB) from a JSON file.
cv::Mat load_handeye_param_from_json(const std::string& filepath);

} // namespace common
} // namespace core

#endif // FILE_UTILS_H
