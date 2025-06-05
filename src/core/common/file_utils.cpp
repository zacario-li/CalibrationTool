#include "file_utils.h"
#include <fstream>
#include <iostream> // For std::cerr
#include <nlohmann/json.hpp>
#include <opencv2/core/mat.hpp>

using json = nlohmann::json;

namespace core {
namespace common {

namespace { // Anonymous namespace for helper functions

// Helper to parse a JSON array into a cv::Mat (single row or column vector)
cv::Mat json_array_to_cv_mat_vector(const json& j_arr, int expected_size) {
    if (!j_arr.is_array() || j_arr.size() != expected_size) {
        throw std::runtime_error("JSON array has incorrect size or is not an array.");
    }
    cv::Mat mat(1, expected_size, CV_64F);
    for (int i = 0; i < expected_size; ++i) {
        mat.at<double>(0, i) = j_arr[i].get<double>();
    }
    return mat;
}

// Helper to parse a JSON 2D array (array of arrays) into a cv::Mat
cv::Mat json_array_to_cv_mat_2d(const json& j_arr, int expected_rows, int expected_cols) {
    if (!j_arr.is_array() || j_arr.empty() || j_arr.size() != expected_rows) {
        throw std::runtime_error("JSON array of arrays has incorrect structure or row count.");
    }
    cv::Mat mat(expected_rows, expected_cols, CV_64F);
    for (int r = 0; r < expected_rows; ++r) {
        const auto& row_json = j_arr[r];
        if (!row_json.is_array() || row_json.size() != expected_cols) {
            throw std::runtime_error("JSON inner array has incorrect column count or is not an array.");
        }
        for (int c = 0; c < expected_cols; ++c) {
            mat.at<double>(r, c) = row_json[c].get<double>();
        }
    }
    return mat;
}

} // Anonymous namespace

CameraParams load_camera_param_from_json(
    const std::string& filepath,
    bool load_stereo_extrinsics,
    int camera_id_for_stereo,
    bool transpose_intrinsic,
    bool load_image_size) {

    CameraParams params;
    std::ifstream ifs(filepath);
    if (!ifs.is_open()) {
        std::cerr << "Error: Could not open file: " << filepath << std::endl;
        // Return empty params or throw exception
        return params;
    }

    json j;
    try {
        ifs >> j;
    } catch (json::parse_error& e) {
        std::cerr << "Error: JSON parsing error in file " << filepath << ": " << e.what() << std::endl;
        return params;
    }

    try {
        std::string main_element_name;
        if (j.contains("CameraParameters")) {
            main_element_name = "CameraParameters";
        } else if (camera_id_for_stereo == 0 && j.contains("CameraParameters1")) {
            main_element_name = "CameraParameters1";
        } else if (camera_id_for_stereo == 1 && j.contains("CameraParameters2")) {
            main_element_name = "CameraParameters2";
        } else {
            std::cerr << "Error: JSON file " << filepath << " does not contain expected camera parameter root elements." << std::endl;
            return params;
        }

        const auto& cam_params_json = j.at(main_element_name);

        // Intrinsic Matrix
        if (cam_params_json.contains("IntrinsicMatrix")) {
            params.intrinsic_matrix = json_array_to_cv_mat_2d(cam_params_json.at("IntrinsicMatrix"), 3, 3);
            if (transpose_intrinsic) {
                cv::transpose(params.intrinsic_matrix, params.intrinsic_matrix);
            }
        } else {
            std::cerr << "Warning: 'IntrinsicMatrix' not found in " << main_element_name << " in " << filepath << std::endl;
        }

        // Distortion Coefficients
        // Python code combines RadialDistortion (k1, k2, k3) and TangentialDistortion (p1, p2)
        // into [k1, k2, p1, p2, k3]
        if (cam_params_json.contains("RadialDistortion") && cam_params_json.contains("TangentialDistortion")) {
            const auto& rd_json = cam_params_json.at("RadialDistortion");
            const auto& td_json = cam_params_json.at("TangentialDistortion");

            if (rd_json.is_array() && rd_json.size() == 3 && td_json.is_array() && td_json.size() == 2) {
                params.distortion_coeffs = cv::Mat(1, 5, CV_64F);
                params.distortion_coeffs.at<double>(0, 0) = rd_json[0].get<double>(); // k1
                params.distortion_coeffs.at<double>(0, 1) = rd_json[1].get<double>(); // k2
                params.distortion_coeffs.at<double>(0, 2) = td_json[0].get<double>(); // p1
                params.distortion_coeffs.at<double>(0, 3) = td_json[1].get<double>(); // p2
                params.distortion_coeffs.at<double>(0, 4) = rd_json[2].get<double>(); // k3
            } else {
                std::cerr << "Warning: 'RadialDistortion' or 'TangentialDistortion' has incorrect format in " << filepath << std::endl;
            }
        } else {
            std::cerr << "Warning: 'RadialDistortion' or 'TangentialDistortion' not found in " << main_element_name << " in " << filepath << std::endl;
        }

        if (load_image_size && j.contains("ImageShape")) {
            const auto& shape_json = j.at("ImageShape");
            if (shape_json.is_array() && shape_json.size() == 2) {
                params.image_size = cv::Size(shape_json[0].get<int>(), shape_json[1].get<int>());
                params.is_image_size_loaded = true;
            } else {
                 std::cerr << "Warning: 'ImageShape' has incorrect format in " << filepath << std::endl;
            }
        }


        if (load_stereo_extrinsics) {
            if (j.contains("RotationOfCamera2") && j.contains("TranslationOfCamera2")) {
                params.rotation_matrix_cam2 = json_array_to_cv_mat_2d(j.at("RotationOfCamera2"), 3, 3);
                // Translation in JSON is often [tx, ty, tz], store as 3x1 vector
                cv::Mat trans_vec_row = json_array_to_cv_mat_vector(j.at("TranslationOfCamera2"), 3);
                params.translation_vector_cam2 = trans_vec_row.reshape(0, 3); // Reshape to 3x1
                params.is_stereo_param_loaded = true;
            } else {
                std::cerr << "Warning: 'RotationOfCamera2' or 'TranslationOfCamera2' not found for stereo loading in " << filepath << std::endl;
            }
        }

    } catch (const json::exception& e) {
        std::cerr << "Error: JSON access error in file " << filepath << ": " << e.what() << std::endl;
        // Reset params to indicate failure
        params = CameraParams();
    } catch (const std::runtime_error& e) {
        std::cerr << "Error: Data conversion error in file " << filepath << ": " << e.what() << std::endl;
        params = CameraParams();
    }

    return params;
}

cv::Mat load_handeye_param_from_json(const std::string& filepath) {
    cv::Mat handeye_matrix;
    std::ifstream ifs(filepath);
    if (!ifs.is_open()) {
        std::cerr << "Error: Could not open file: " << filepath << std::endl;
        return handeye_matrix; // Return empty Mat
    }

    json j;
    try {
        ifs >> j;
    } catch (json::parse_error& e) {
        std::cerr << "Error: JSON parsing error in file " << filepath << ": " << e.what() << std::endl;
        return handeye_matrix;
    }

    try {
        std::string root_element = "AXXB"; // Default from Python code
        if (j.contains("AXZB")) {
            root_element = "AXZB";
        } else if (!j.contains("AXXB")) {
             std::cerr << "Error: JSON file " << filepath << " does not contain 'AXXB' or 'AXZB' root element." << std::endl;
            return handeye_matrix;
        }

        if (j.at(root_element).contains("Matrix")) {
            handeye_matrix = json_array_to_cv_mat_2d(j.at(root_element).at("Matrix"), 4, 4);
        } else {
            std::cerr << "Warning: '" << root_element << ".Matrix' not found in " << filepath << std::endl;
        }

    } catch (const json::exception& e) {
        std::cerr << "Error: JSON access error in file " << filepath << ": " << e.what() << std::endl;
        handeye_matrix = cv::Mat(); // Clear matrix on error
    } catch (const std::runtime_error& e) {
        std::cerr << "Error: Data conversion error in file " << filepath << ": " << e.what() << std::endl;
        handeye_matrix = cv::Mat();
    }

    return handeye_matrix;
}

} // namespace common
} // namespace core
