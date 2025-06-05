#include "hand_eye_calibration.h"
#include "transformation_utils.h" // For quaternion_to_rotation_matrix, combine_rt_to_homogeneous, compute_rotation_angle_delta
#include <opencv2/calib3d.hpp>    // For cv::Rodrigues, cv::calibrateHandEye
#include <fstream>
#include <sstream>
#include <iostream> // For std::cerr, std::cout
#include <iomanip>  // For std::setprecision
#include <random>   // For random test translation

namespace core {
namespace calib {

// Helper to parse a line of numbers from a string stream
template<typename T>
std::vector<T> parse_line_to_vector(std::stringstream& ss, size_t expected_count) {
    std::vector<T> values;
    T val;
    size_t count = 0;
    while (ss >> val && count < expected_count) {
        values.push_back(val);
        count++;
        if (ss.peek() == ',' || ss.peek() == ' ') { // Handle comma or space separation
            ss.ignore();
        }
    }
    if (count != expected_count) {
        // Triggered if line has too few or too many values before non-numeric char
        // Or if extraction fails for a type
        throw std::runtime_error("Line does not contain the expected number of numeric values or has invalid format.");
    }
    // Check if there's any trailing non-whitespace data that wasn't parsed
    std::string remaining;
    ss >> remaining;
    if (!remaining.empty() && !std::all_of(remaining.begin(), remaining.end(), ::isspace)) {
         throw std::runtime_error("Trailing non-numeric data found on line or invalid format.");
    }
    return values;
}


bool HandEyeCalibration::load_robot_poses_from_rvec_txt(
    const std::filesystem::path& filepath,
    std::vector<cv::Mat>& R_gripper2base_list,
    std::vector<cv::Mat>& t_gripper2base_list,
    bool convert_translation_to_mm) {

    R_gripper2base_list.clear();
    t_gripper2base_list.clear();

    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file: " << filepath << std::endl;
        return false;
    }

    std::string line;
    int line_num = 0;
    while (std::getline(file, line)) {
        line_num++;
        std::stringstream ss(line);
        try {
            std::vector<double> values = parse_line_to_vector<double>(ss, 6); // x, y, z, rx, ry, rz

            cv::Mat t_vec = (cv::Mat_<double>(3, 1) << values[0], values[1], values[2]);
            if (convert_translation_to_mm) {
                t_vec *= 1000.0; // Convert meters to mm
            }

            cv::Mat r_vec = (cv::Mat_<double>(3, 1) << values[3], values[4], values[5]);
            cv::Mat R_mat;
            cv::Rodrigues(r_vec, R_mat);

            R_gripper2base_list.push_back(R_mat);
            t_gripper2base_list.push_back(t_vec);

        } catch (const std::runtime_error& e) {
            std::cerr << "Error parsing line " << line_num << " in file " << filepath << ": " << e.what() << " Line content: \"" << line << "\"" << std::endl;
            // Continue to next line or return false? For now, skip this line.
            // To be stricter, you might want to return false here.
        }
    }
    return !R_gripper2base_list.empty();
}

bool HandEyeCalibration::load_robot_poses_from_quat_csv(
    const std::filesystem::path& filepath,
    std::vector<cv::Mat>& R_gripper2base_list,
    std::vector<cv::Mat>& t_gripper2base_list,
    bool sensor_only_rotation,
    bool random_test_translation) {

    R_gripper2base_list.clear();
    t_gripper2base_list.clear();

    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file: " << filepath << std::endl;
        return false;
    }

    std::string line;
    int line_num = 0;
    bool header_skipped = false;

    std::mt19937 rng; // Mersenne Twister random number generator
    std::uniform_real_distribution<double> dist(0.0, 1.0); // Uniform distribution [0,1)
    if (random_test_translation) {
         std::random_device rd;
         rng.seed(rd());
    }


    while (std::getline(file, line)) {
        line_num++;
        if (!header_skipped) { // Skip header line (q0,qx,qy,qz,tx,ty,tz)
            header_skipped = true;
            continue;
        }

        std::stringstream ss(line);
        try {
            std::vector<double> values = parse_line_to_vector<double>(ss, 7); // q0,qx,qy,qz,tx,ty,tz

            cv::Vec4d quat(values[0], values[1], values[2], values[3]); // Assuming w, x, y, z order
            cv::Mat R_mat = quaternion_to_rotation_matrix(quat);
            R_gripper2base_list.push_back(R_mat);

            cv::Mat t_vec;
            if (sensor_only_rotation) {
                if (random_test_translation) {
                    t_vec = (cv::Mat_<double>(3,1) << dist(rng), dist(rng), dist(rng));
                } else {
                    t_vec = (cv::Mat_<double>(3,1) << 1.0, 1.0, 1.0); // Dummy translation
                }
            } else {
                t_vec = (cv::Mat_<double>(3,1) << values[4], values[5], values[6]);
            }
            t_gripper2base_list.push_back(t_vec);

        } catch (const std::runtime_error& e) {
            std::cerr << "Error parsing line " << line_num << " in file " << filepath << ": " << e.what() << " Line content: \"" << line << "\"" << std::endl;
            // Continue or return false
        }
    }
    return !R_gripper2base_list.empty();
}


HandEyeResult HandEyeCalibration::calibrate_axxb(
    const std::vector<cv::Mat>& R_gripper2base,
    const std::vector<cv::Mat>& t_gripper2base,
    const std::vector<cv::Mat>& R_target2cam,
    const std::vector<cv::Mat>& t_target2cam,
    cv::HandEyeCalibrationMethod method) {

    HandEyeResult result;
    result.status = core::common::CalibErrType::CAL_DATA_SIZE_NOT_MATCH; // Default error

    if (R_gripper2base.size() != t_gripper2base.size() ||
        R_target2cam.size() != t_target2cam.size() ||
        R_gripper2base.size() != R_target2cam.size() ||
        R_gripper2base.empty()) {
        std::cerr << "Error: Input data size mismatch or empty lists for AX=XB calibration." << std::endl;
        std::cerr << "Sizes: R_gripper2base=" << R_gripper2base.size()
                  << ", t_gripper2base=" << t_gripper2base.size()
                  << ", R_target2cam=" << R_target2cam.size()
                  << ", t_target2cam=" << t_target2cam.size() << std::endl;
        return result;
    }

    cv::Mat R_cam2gripper, t_cam2gripper;
    cv::calibrateHandEye(
        R_gripper2base, t_gripper2base, // Robot gripper to base transformations
        R_target2cam, t_target2cam,     // Calibration target to camera transformations
        R_cam2gripper, t_cam2gripper,   // Output: Camera to gripper transformation
        method
    );

    if (R_cam2gripper.empty() || t_cam2gripper.empty()) {
        std::cerr << "Error: cv::calibrateHandEye failed to compute a solution." << std::endl;
        result.status = core::common::CalibErrType::CAL_CORNER_DET_ERR; // Or a more generic calibration failure
        return result;
    }

    result.X = combine_rt_to_homogeneous(R_cam2gripper, t_cam2gripper);
    result.status = core::common::CalibErrType::CAL_OK;

    // Perform validation
    try {
        result.rotation_error = compute_rotation_error(R_gripper2base, R_target2cam, R_cam2gripper);
        result.translation_error = compute_translation_error(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper, t_cam2gripper);
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV Exception during HandEye validation: " << e.what() << std::endl;
        // Errors are already set to -1.0, status remains CAL_OK as calibration itself succeeded.
    } catch (const std::exception& e) {
        std::cerr << "Standard Exception during HandEye validation: " << e.what() << std::endl;
    }


    return result;
}

// Private validation methods (can be adapted from Python's valaxxb, te, re)
double HandEyeCalibration::compute_rotation_error(
    const std::vector<cv::Mat>& R_gripper2base_list, // A_i list
    const std::vector<cv::Mat>& R_target2cam_list,   // B_i list
    const cv::Mat& R_cam2gripper) {                  // X (rotation part)

    if (R_gripper2base_list.size() < 2 || R_gripper2base_list.size() != R_target2cam_list.size()) {
        std::cerr << "Warning (compute_rotation_error): Insufficient data for pairwise comparison." << std::endl;
        return -1.0;
    }

    double total_angle_error = 0;
    int valid_pairs = 0;

    // Python code: AX = XB -> A = RT_g2n_2^{-1} @ RT_g2n_1, B = RT_b2c_2 @ RT_b2c_1^{-1}
    // Here, R_gripper2base is G_H_B (gripper in base frame), R_target2cam is T_H_C (target in camera frame)
    // We need pairs to form A_ij = (G_H_B_j)^-1 * (G_H_B_i) and B_ij = (T_H_C_j) * (T_H_C_i)^-1
    // Then check (A_ij * X) vs (X * B_ij)
    // Or, (G_H_B_i)^-1 * X * T_H_C_i should be constant for all i (this is X^-1 * A * X = B form)
    // Let's follow Python's valaxxb approach which calculates δR = {R_X}{R_B}({R_A}{R_X})^{-1}
    // A = RT_g2n_1 (let's call it H_g1b - gripper1 to base)
    // B = RT_b2c_1 (let's call it H_t1c - target1 to camera)
    // X = RT_c2g   (H_c2g - camera to gripper)
    // For a second pose: H_g2b, H_t2c
    // A_prime = H_g2b * H_g1b.inv()  (gripper2 relative to gripper1, in base frame)
    // B_prime = H_t2c.inv() * H_t1c  (target1 relative to target2, in camera frame)
    // We want to check: A_prime * X_rot = X_rot * B_prime_inv
    // This is equivalent to: (H_g2b * H_g1b.inv()) * R_c2g = R_c2g * (H_t1c.inv() * H_t2c)
    // Python's valaxxb:
    // AX.append(a @ RT_c2g) where a = np.linalg.inv(RT_g2n_2) @ RT_g2n_1
    // XB.append(RT_c2g @ b) where b = RT_b2c_2 @ np.linalg.inv(RT_b2c_1)
    // This means: (H_g2b.inv() * H_g1b) @ H_c2g  vs  H_c2g @ (H_t2c * H_t1c.inv())

    for (size_t i = 0; i < R_gripper2base_list.size(); ++i) {
        for (size_t j = i + 1; j < R_gripper2base_list.size(); ++j) {
            // Pose i
            const cv::Mat& R_g1b = R_gripper2base_list[i]; // Gripper_i H Base
            const cv::Mat& R_t1c = R_target2cam_list[i];   // Target_i H Camera

            // Pose j
            const cv::Mat& R_g2b = R_gripper2base_list[j]; // Gripper_j H Base
            const cv::Mat& R_t2c = R_target2cam_list[j];   // Target_j H Camera

            cv::Mat R_g1b_inv = R_g1b.inv();
            cv::Mat R_t1c_inv = R_t1c.inv();

            cv::Mat A_rot = R_g2b.inv() * R_g1b; // Gripper_i H Gripper_j (in base frame)
            cv::Mat B_rot = R_t2c * R_t1c.inv(); // Target_j H Target_i (in camera frame)

            cv::Mat AX_rot = A_rot * R_cam2gripper;
            cv::Mat XB_rot = R_cam2gripper * B_rot;

            // delta_R = XB_rot * AX_rot.inv();
            // double trace = cv::trace(delta_R);
            // double angle_rad = std::acos(std::max(-1.0, std::min(1.0, (trace - 1.0) / 2.0)));
            // total_angle_error += angle_rad * (180.0 / CV_PI);
            total_angle_error += compute_rotation_angle_delta(AX_rot, XB_rot); // Using the utility
            valid_pairs++;
        }
    }

    return (valid_pairs > 0) ? (total_angle_error / valid_pairs) : -1.0;
}


double HandEyeCalibration::compute_translation_error(
    const std::vector<cv::Mat>& R_gripper2base_list, // R_A_list
    const std::vector<cv::Mat>& t_gripper2base_list, // t_A_list
    const std::vector<cv::Mat>& R_target2cam_list,   // R_B_list
    const std::vector<cv::Mat>& t_target2cam_list,   // t_B_list
    const cv::Mat& R_cam2gripper,                    // R_X
    const cv::Mat& t_cam2gripper) {                  // t_X

    if (R_gripper2base_list.size() < 2 || R_gripper2base_list.size() != t_gripper2base_list.size() ||
        R_gripper2base_list.size() != R_target2cam_list.size() || R_gripper2base_list.size() != t_target2cam_list.size()) {
        std::cerr << "Warning (compute_translation_error): Insufficient data for pairwise comparison." << std::endl;
        return -1.0;
    }

    double total_translation_error_norm = 0;
    int valid_pairs = 0;

    // Python: err_te = 1/N \sum || (R_A_i @ t_X) + t_A_i - (R_X @ t_B_i) - t_X ||
    // Where A_i relates two gripper poses (e.g., H_g2b.inv() * H_g1b),
    // and B_i relates two target poses (e.g., H_t2c * H_t1c.inv()).
    // R_A_i is rotation part of A_i, t_A_i is translation part of A_i.
    // R_X is R_cam2gripper, t_X is t_cam2gripper.

    for (size_t i = 0; i < R_gripper2base_list.size(); ++i) {
        for (size_t j = i + 1; j < R_gripper2base_list.size(); ++j) {
            // Pose i (H_g1b, H_t1c)
            const cv::Mat& R_g1b = R_gripper2base_list[i];
            const cv::Mat& t_g1b = t_gripper2base_list[i];
            const cv::Mat& R_t1c = R_target2cam_list[i];
            const cv::Mat& t_t1c = t_target2cam_list[i];

            // Pose j (H_g2b, H_t2c)
            const cv::Mat& R_g2b = R_gripper2base_list[j];
            const cv::Mat& t_g2b = t_gripper2base_list[j];
            const cv::Mat& R_t2c = R_target2cam_list[j];
            const cv::Mat& t_t2c = t_target2cam_list[j];

            // A_ij = H_g1b_inv * H_g2b (transformation from gripper pose j to gripper pose i, in base frame)
            // Note: Python code used A = H_g2b_inv * H_g1b. Let's stick to that for consistency of error formula.
            // A_ij = H_g2b_inv * H_g1b
            cv::Mat R_g2b_inv = R_g2b.inv();
            cv::Mat R_A_ij = R_g2b_inv * R_g1b;
            cv::Mat t_A_ij = R_g2b_inv * (t_g1b - t_g2b);

            // B_ij = H_t2c * H_t1c_inv (transformation from target pose i to target pose j, in camera frame)
            cv::Mat R_t1c_inv = R_t1c.inv();
            cv::Mat R_B_ij = R_t2c * R_t1c_inv;
            cv::Mat t_B_ij = t_t2c - R_t2c * R_t1c_inv * t_t1c;

            // Error term: (R_A_ij * t_cam2gripper) + t_A_ij - (R_cam2gripper * t_B_ij) - t_cam2gripper
            cv::Mat term1 = R_A_ij * t_cam2gripper;
            cv::Mat term2 = t_A_ij;
            cv::Mat term3 = R_cam2gripper * t_B_ij;
            cv::Mat term4 = t_cam2gripper;

            cv::Mat error_vec = term1 + term2 - term3 - term4;
            total_translation_error_norm += cv::norm(error_vec);
            valid_pairs++;
        }
    }
    return (valid_pairs > 0) ? (total_translation_error_norm / valid_pairs) : -1.0;
}


} // namespace calib
} // namespace core
