#ifndef HAND_EYE_CALIBRATION_H
#define HAND_EYE_CALIBRATION_H

#include "calib_types.h"
#include <opencv2/core/mat.hpp>
#include <vector>
#include <string>
#include <filesystem>

namespace core {
namespace calib {

class HandEyeCalibration {
public:
    HandEyeCalibration() = default;

    // Loads robot poses (gripper_H_base, or A matrices for AX=XB)
    // Each row: x y z rx ry rz (translation in meters, rotation as Rodrigues vector)
    // Output: R_gripper2base_list, t_gripper2base_list (translation in mm)
    static bool load_robot_poses_from_rvec_txt(
        const std::filesystem::path& filepath,
        std::vector<cv::Mat>& R_gripper2base_list,
        std::vector<cv::Mat>& t_gripper2base_list,
        bool convert_translation_to_mm = true);

    // Loads robot poses (gripper_H_base or A matrices for AX=XB)
    // CSV file with header: q0,qx,qy,qz,tx,ty,tz (quaternion, translation in original units)
    // Output: R_gripper2base_list, t_gripper2base_list
    static bool load_robot_poses_from_quat_csv(
        const std::filesystem::path& filepath,
        std::vector<cv::Mat>& R_gripper2base_list,
        std::vector<cv::Mat>& t_gripper2base_list,
        bool sensor_only_rotation = false, // If true, translation part from CSV is ignored / filled with dummy
        bool random_test_translation = false // If sensor_only_rotation is true, fill with random translation
    );

    // Performs AX=XB hand-eye calibration.
    // R_gripper2base: List of rotation matrices (gripper frame relative to robot base).
    // t_gripper2base: List of translation vectors (gripper frame relative to robot base).
    // R_target2cam: List of rotation matrices (calibration target frame relative to camera).
    // t_target2cam: List of translation vectors (calibration target frame relative to camera).
    // Method: e.g., cv::CALIB_HAND_EYE_TSAI, cv::CALIB_HAND_EYE_PARK, etc.
    HandEyeResult calibrate_axxb(
        const std::vector<cv::Mat>& R_gripper2base,
        const std::vector<cv::Mat>& t_gripper2base,
        const std::vector<cv::Mat>& R_target2cam,
        const std::vector<cv::Mat>& t_target2cam,
        cv::HandEyeCalibrationMethod method);

    // AX=ZB calibration (stub for now)
    // HandEyeResult calibrate_axzb(...);

private:
    // Validation methods (internal to calibrate_axxb or can be public utilities)
    // Computes rotation error based on AX=XB, returns mean angle error in degrees
    double compute_rotation_error(
        const std::vector<cv::Mat>& R_gripper2base,
        const std::vector<cv::Mat>& R_target2cam,
        const cv::Mat& R_cam2gripper);

    // Computes translation error based on AX=XB, returns mean L2 norm error
    double compute_translation_error(
        const std::vector<cv::Mat>& R_gripper2base,
        const std::vector<cv::Mat>& t_gripper2base,
        const std::vector<cv::Mat>& R_target2cam,
        const std::vector<cv::Mat>& t_target2cam,
        const cv::Mat& R_cam2gripper,
        const cv::Mat& t_cam2gripper);
};

} // namespace calib
} // namespace core

#endif // HAND_EYE_CALIBRATION_H
