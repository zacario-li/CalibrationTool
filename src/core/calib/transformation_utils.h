#ifndef TRANSFORMATION_UTILS_H
#define TRANSFORMATION_UTILS_H

#include <opencv2/core/mat.hpp>
#include <opencv2/calib3d.hpp> // For Rodrigues
#include <vector>

namespace core {
namespace calib {

// Combines a 3x3 rotation matrix and a 3x1 translation vector (or 1x3)
// into a 4x4 homogeneous transformation matrix.
cv::Mat combine_rt_to_homogeneous(const cv::Mat& rotation_matrix, const cv::Mat& translation_vector);

// Converts a 3x3 rotation matrix to a 4x1 quaternion (w, x, y, z).
// OpenCV's Rodrigues function can convert rotation matrix to rotation vector,
// then custom logic or a library like Eigen can convert to quaternion.
// This implementation will provide a basic conversion.
cv::Vec4d rotation_matrix_to_quaternion(const cv::Mat& rotation_matrix);

// Converts a 4x1 quaternion (w, x, y, z) to a 3x3 rotation matrix.
cv::Mat quaternion_to_rotation_matrix(const cv::Vec4d& quaternion);

// Computes the angle of rotation from a delta rotation matrix.
// delta_rotation = R_final * R_initial.inv()
// Returns angle in degrees.
double compute_rotation_angle_delta(const cv::Mat& r_initial, const cv::Mat& r_final);

} // namespace calib
} // namespace core

#endif // TRANSFORMATION_UTILS_H
