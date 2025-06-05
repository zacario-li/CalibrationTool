#ifndef CALIB_BOARD_H
#define CALIB_BOARD_H

#include "calib_types.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/objdetect/charuco_board.hpp>
#include <vector>
#include <string>
#include <filesystem> // For directory iteration

namespace core {
namespace calib {

class CalibBoard {
public:
    CalibBoard(
        int board_rows, // Number of inner corners vertically (e.g., 6 for a 7xN board)
        int board_cols, // Number of inner corners horizontally (e.g., 9 for a Nx10 board)
        float square_size_mm,
        CalibPatternType pattern_type = CalibPatternType::CHESSBOARD,
        bool use_custom_detector = false, // Corresponds to use_libcbdet
        // Charuco specific parameters (if pattern_type is CHARUCO)
        int charuco_dict_id = cv::aruco::DICT_4X4_1000,
        float charuco_square_size_aruco_marker_ratio = 0.5f // e.g. marker is 2.5cm, square is 5cm -> ratio = 0.5
    );

    // Finds corners in a single image
    ImageCornersResult find_corners_in_image(const cv::Mat& image) const;

    // Finds corners in multiple images (can be parallelized internally)
    std::vector<ImageCornersResult> find_corners_in_images(
        const std::vector<std::filesystem::path>& image_paths,
        bool use_multithreading = true
    ) const;


    // Calibrates a single camera
    MonoCalibResult mono_calibrate(
        const std::vector<std::filesystem::path>& image_paths,
        bool use_multithreading = true,
        int opencv_calib_flags = 0 // e.g. CALIB_FIX_ASPECT_RATIO, etc.
    );

    // Stereo calibration (stub for now)
    StereoCalibResult stereo_calibrate(
        const std::vector<std::filesystem::path>& left_image_paths,
        const std::vector<std::filesystem::path>& right_image_paths,
        bool use_multithreading = true,
        int opencv_calib_flags = 0
    );

    // Estimates pose of the calibration board relative to the camera
    ImagePoseResult estimate_pose(
        const ImageCornersResult& detected_corners, // Use pre-detected corners
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs
    ) const;

    ImagePoseResult estimate_pose_from_image( // Finds corners then estimates pose
        const cv::Mat& image,
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs
    ) const;


    // Drawing functions (stubs for now)
    cv::Mat draw_corners_on_image(const cv::Mat& image, const std::vector<cv::Point2f>& corners, bool pattern_found) const;
    cv::Mat draw_reprojection_arrows(const cv::Mat& image, const std::vector<cv::Point2f>& detected_corners, const std::vector<cv::Point2f>& reprojected_corners) const;
    cv::Mat draw_axes_on_image(const cv::Mat& image, const cv::Mat& rvec, const cv::Mat& tvec, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs, float axis_length_mm) const;


    // Getters
    int get_board_rows() const { return board_rows_; }
    int get_board_cols() const { return board_cols_; }
    float get_square_size_mm() const { return square_size_mm_; }
    const std::vector<cv::Point3f>& get_object_points() const { return object_points_;}
    const cv::Ptr<cv::aruco::CharucoBoard>& get_charuco_board() const { return charuco_board_; }


private:
    int board_rows_;            // Number of inner corners along rows
    int board_cols_;            // Number of inner corners along columns
    float square_size_mm_;      // Size of a square in millimeters
    CalibPatternType pattern_type_;
    bool use_custom_detector_;  // If true, uses custom detector (placeholder)

    std::vector<cv::Point3f> object_points_; // Standard 3D points for chessboard/circles grid
    cv::Ptr<cv::aruco::CharucoBoard> charuco_board_; // For Charuco pattern
    cv::Ptr<cv::aruco::DetectorParameters> charuco_detector_params_; // Parameters for charuco detection
    cv::Ptr<cv::aruco::Dictionary> charuco_dictionary_;


    // Helper for parallel corner detection
    ImageCornersResult process_single_image_for_corners(const std::filesystem::path& image_path) const;

    // Helper for parallel stereo corner detection (if needed, similar to mono)
    // StereoImageCornersResult process_single_stereo_pair_for_corners(...) const;
};

} // namespace calib
} // namespace core

#endif // CALIB_BOARD_H
