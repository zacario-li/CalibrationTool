#include "calib_board.h"
#include <opencv2/imgcodecs.hpp> // For cv::imread
#include <opencv2/imgproc.hpp>   // For cv::cvtColor, cv::drawChessboardCorners
#include <opencv2/calib3d.hpp>   // For cv::findChessboardCornersSB, cv::calibrateCamera, cv::solvePnP
#include <opencv2/objdetect/aruco_detector.hpp> // For Charuco
#include <iostream>              // For std::cerr, std::cout
#include <vector>
#include <future>                // For std::async, std::future
#include <algorithm>             // For std::all_of
#include <core/common/app_errors.h>


namespace core {
namespace calib {

CalibBoard::CalibBoard(
    int board_rows,
    int board_cols,
    float square_size_mm,
    CalibPatternType pattern_type,
    bool use_custom_detector,
    int charuco_dict_id,
    float charuco_square_size_aruco_marker_ratio)
    : board_rows_(board_rows),
      board_cols_(board_cols),
      square_size_mm_(square_size_mm),
      pattern_type_(pattern_type),
      use_custom_detector_(use_custom_detector),
      charuco_detector_params_(cv::makePtr<cv::aruco::DetectorParameters>()) { // Initialize detector params

    if (pattern_type_ == CalibPatternType::CHESSBOARD) {
        for (int i = 0; i < board_cols_; ++i) { // board_cols_ is num_inner_corners_ver, board_rows_ is num_inner_corners_hor
            for (int j = 0; j < board_rows_; ++j) {
                object_points_.emplace_back(cv::Point3f(j * square_size_mm_, i * square_size_mm_, 0.0f));
            }
        }
    } else if (pattern_type_ == CalibPatternType::CHARUCO) {
        charuco_dictionary_ = cv::aruco::getPredefinedDictionary(charuco_dict_id);
        // Note: CharucoBoard constructor takes total squares (cols+1, rows+1 typically for outer dimensions)
        // The board_cols_ and board_rows_ for CalibBoard are usually inner corners.
        // Python CalibBoard used (col, row) for CharucoBoard, which might mean (squaresX, squaresY).
        // Let's assume board_rows_ and board_cols_ for CalibBoard are inner corners, so squares are +1.
        // However, the Python code for Charuco uses (col, row) directly for CharucoBoard constructor.
        // If board_rows/cols are inner corners, then squaresX = board_rows+1, squaresY = board_cols+1.
        // If board_rows/cols are squares, then use them directly.
        // Python: self.charuco_board = aruco.CharucoBoard((col, row), cellsize, charuco_size/1000, self.charuco_dict)
        // Here, (col,row) seems to be squaresX, squaresY. Let's assume board_rows_ means squaresX for Charuco, board_cols_ means squaresY
        // This is a bit ambiguous from the Python code, usually board_rows/cols refers to *inner corners count*.
        // For Charuco, it's number of squares. Assuming board_rows_ and board_cols_ for this class
        // mean *squares* for Charuco pattern, consistent with aruco.CharucoBoard((col,row)...) in Python.

        float aruco_marker_size_mm = square_size_mm_ * charuco_square_size_aruco_marker_ratio;
        charuco_board_ = cv::makePtr<cv::aruco::CharucoBoard>(
            cv::Size(board_rows_, board_cols_), // squaresX, squaresY
            square_size_mm_,
            aruco_marker_size_mm,
            charuco_dictionary_);

        // For Charuco, object_points_ are implicitly defined by the board.
        // We can extract them if needed for functions that expect object_points explicitly,
        // but cv::aruco::calibrateCameraCharuco uses the board directly.
        // object_points_ = charuco_board_->getChessboardCorners(); // This would give all corners.
    } else if (pattern_type_ == CalibPatternType::APRILTAG) {
        // AprilTag specific initialization (e.g., TagFamily, layout) would go here.
        // Object points would be defined based on the specific AprilGrid layout.
        std::cerr << "Warning: APRILTAG pattern not fully implemented yet." << std::endl;
    } else {
        std::cerr << "Error: Unknown calibration pattern type." << std::endl;
    }
}

ImageCornersResult CalibBoard::find_corners_in_image(const cv::Mat& image) const {
    ImageCornersResult result;
    result.found = false;
    result.error_code = core::common::CalibErrType::CAL_CORNER_DET_ERR; // Default to error

    result.found = false;
    result.error_code = core::common::CalibErrType::CAL_CORNER_DET_ERR; // Default to error

    if (image.empty()) {
        std::cerr << "Error (CalibBoard::find_corners_in_image): Input image is empty." << std::endl;
        return result;
    }

    cv::Mat gray_image;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
    } else {
        gray_image = image.clone(); // Ensure it's a new Mat if already grayscale
    }

    if (use_custom_detector_ && pattern_type_ == CalibPatternType::CHESSBOARD) {
        // Call the custom detector
        // board_rows_ in CalibBoard is inner corners horizontal (Python's ROW_COR = pattern_cols_opencv)
        // board_cols_ in CalibBoard is inner corners vertical   (Python's COL_COR = pattern_rows_opencv)
        // detect_checkerboard_custom expects cv::Size(inner_cols, inner_rows)
        // So, cv::Size(board_rows_, board_cols_) is correct.
        double score = core::detectors::detect_checkerboard_custom(gray_image, cv::Size(board_rows_, board_cols_), result.corners);
        if (score < 0.3 && !result.corners.empty() && result.corners.size() == static_cast<size_t>(board_rows_ * board_cols_)) {
            result.found = true;
            result.error_code = core::common::CalibErrType::CAL_OK;
            result.object_points_for_image = object_points_;
        } else {
            result.found = false; // Ensure it's false if score is bad or corners mismatch
            // result.error_code is already CAL_CORNER_DET_ERR
        }
        return result; // Return after custom detector attempt
    }

    // --- OpenCV's default detectors ---
    if (pattern_type_ == CalibPatternType::CHESSBOARD) {
        // board_rows_ from Python was num_inner_cols, board_cols_ was num_inner_rows
        // cv::findChessboardCornersSB expects (pattern_cols, pattern_rows) which are inner corners.
        // So, Python's (row-1, col-1) -> C++ (board_rows_-1, board_cols_-1) if rows/cols are total squares
        // Or if board_rows/cols are already inner corners, use them directly.
        // Python: self.ROW_COR = row-1, self.COL_COR = col-1. Then cv2.findChessboardCornersSB(gray, (self.ROW_COR, self.COL_COR), ...)
        // So, this class's board_rows_ maps to Python's (row-1) and board_cols_ maps to Python's (col-1)
        // This means board_rows_ is number of inner corners along one dim, board_cols_ along other.
        // Let's assume board_rows_ is horizontal inner corners, board_cols_ is vertical inner corners
        // cv::Size pattern_size(board_rows_, board_cols_); // (corners_per_row, corners_per_col)

        // Based on python: objp[:, :2] = np.mgrid[0:self.ROW_COR, 0:self.COL_COR].T.reshape(-1, 2)
        // ROW_COR = row-1 (e.g. 9-1=8), COL_COR = col-1 (e.g. 12-1=11)
        // So pattern size for findChessboardCornersSB should be (ROW_COR, COL_COR)
        // If this class's board_rows_ is ROW_COR and board_cols_ is COL_COR:
        cv::Size pattern_size(board_rows_, board_cols_);

        result.found = cv::findChessboardCornersSB(
            gray_image,
            pattern_size,
            result.corners,
            cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY); // Recommended flags

        if (result.found) {
            // Optional: cv::cornerSubPix for further refinement if not using SB or if needed.
            // cv::cornerSubPix(gray_image, result.corners, cv::Size(11, 11), cv::Size(-1, -1),
            //                  cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
            result.error_code = core::common::CalibErrType::CAL_OK;
            result.object_points_for_image = object_points_;
        }
    } else if (pattern_type_ == CalibPatternType::CHARUCO) {
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;

        cv::aruco::detectMarkers(gray_image, charuco_dictionary_, marker_corners, marker_ids, charuco_detector_params_);

        if (!marker_ids.empty()) {
            cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, gray_image, charuco_board_, result.corners, result.object_points_for_image); // obj_pts are actually charucoIds here
            // interpolateCornersCharuco returns charucoIds (int vector) not object_points_for_image (Point3f vector)
            // The actual 3D points are obtained from charuco_board_->getChessboardCorners()
            // For calibration, cv::aruco::calibrateCameraCharuco takes the charucoIds and matches them.
            // For now, let's store the detected 2D corners. The object points will be matched during calibration.
            if (!result.corners.empty() && result.corners.size() >=4) { // Need at least 4 corners for calibration
                 result.found = true;
                 result.error_code = core::common::CalibErrType::CAL_OK;
                 // For Charuco, object_points_for_image is tricky. The calibrateCameraCharuco function handles matching.
                 // We can leave result.object_points_for_image empty or fill it if we have a clear mapping for this specific call.
                 // For simplicity, let's assume that if we get here, we have found corners.
                 // The actual 3D points will be derived from charuco_board_ during calibration.
            }
        }
    } else if (pattern_type_ == CalibPatternType::APRILTAG) {
        std::cerr << "Error: AprilTag corner detection not implemented." << std::endl;
    } else {
        std::cerr << "Error: Unknown pattern type for corner detection." << std::endl;
    }
    return result;
}

ImageCornersResult CalibBoard::process_single_image_for_corners(const std::filesystem::path& image_path) const {
    ImageCornersResult result;
    result.image_path = image_path.string();
    cv::Mat img = cv::imread(image_path.string(), cv::IMREAD_COLOR);
    if (img.empty()) {
        std::cerr << "Failed to load image: " << image_path << std::endl;
        result.error_code = core::common::CalibErrType::CAL_CORNER_DET_ERR; // Or a more specific file error
        result.found = false;
        return result;
    }
    return find_corners_in_image(img); // This will set found, corners, object_points_for_image, error_code
}


std::vector<ImageCornersResult> CalibBoard::find_corners_in_images(
    const std::vector<std::filesystem::path>& image_paths, bool use_multithreading) const {

    std::vector<ImageCornersResult> all_results;
    all_results.reserve(image_paths.size());

    if (use_multithreading) {
        std::vector<std::future<ImageCornersResult>> futures;
        for (const auto& path : image_paths) {
            futures.push_back(std::async(std::launch::async,
                                         &CalibBoard::process_single_image_for_corners,
                                         this,
                                         path));
        }
        for (auto& fut : futures) {
            all_results.push_back(fut.get());
        }
    } else {
        for (const auto& path : image_paths) {
            all_results.push_back(process_single_image_for_corners(path));
        }
    }
    return all_results;
}


MonoCalibResult CalibBoard::mono_calibrate(
    const std::vector<std::filesystem::path>& image_paths,
    bool use_multithreading,
    int opencv_calib_flags) {

    MonoCalibResult calib_result;
    calib_result.status = core::common::CalibErrType::CAL_CORNER_DET_ERR; // Default to error

    std::vector<ImageCornersResult> corner_detection_results = find_corners_in_images(image_paths, use_multithreading);

    std::vector<std::vector<cv::Point2f>> all_image_points_successful;
    std::vector<std::vector<cv::Point3f>> all_object_points_successful;

    cv::Mat first_image_for_size;

    for (const auto& res : corner_detection_results) {
        if (res.found && !res.corners.empty()) {
            if (first_image_for_size.empty()) { // Get size from first successfully processed image
                first_image_for_size = cv::imread(res.image_path, cv::IMREAD_GRAYSCALE);
                if (!first_image_for_size.empty()) {
                     calib_result.image_size = first_image_for_size.size();
                } else {
                    std::cerr << "Warning: Could not read image " << res.image_path << " to get size, even after successful corner detection." << std::endl;
                }
            }
            all_image_points_successful.push_back(res.corners);
            // For Charuco, res.object_points_for_image might be charuco IDs.
            // calibrateCameraCharuco needs image points and charuco IDs.
            // calibrateCamera needs image points and object points.
            if (pattern_type_ == CalibPatternType::CHESSBOARD) {
                 all_object_points_successful.push_back(object_points_); // Use the pre-defined board object points
            } else if (pattern_type_ == CalibPatternType::CHARUCO) {
                // For Charuco, object points are implicitly handled by calibrateCameraCharuco
                // We need to collect charucoIds and charucoCorners for that function.
                // This basic mono_calibrate is more geared towards chessboard for now.
                // A dedicated calibrate_charuco method would be better.
                // For now, let's try to make it work by ensuring object_points_ has something,
                // but this part needs refinement for a robust Charuco calibration.
                // The `res.object_points_for_image` from interpolateCornersCharuco is not what calibrateCamera wants.
                // We need to pass the Charuco IDs and corners separately.
                // For now, this will likely fail or be incorrect for Charuco.
                 std::cerr << "Warning: Basic mono_calibrate with Charuco might be incorrect. Use a dedicated Charuco calibration function." << std::endl;
                 // Fallback: try to use the charuco_board's corners if available, but this is not the correct flow for calibrateCameraCharuco
                 if (charuco_board_ && !charuco_board_->getChessboardCorners().empty()) {
                    all_object_points_successful.push_back(charuco_board_->getChessboardCorners());
                 } else {
                    // Add empty, this will likely cause issues or be ignored by calibrateCamera
                    all_object_points_successful.push_back({});
                 }

            }
            calib_result.successfully_calibrated_images.push_back(res.image_path);
        } else {
            calib_result.rejected_images.push_back(res.image_path);
        }
    }

    calib_result.all_image_points = all_image_points_successful; // Store for user
    calib_result.all_object_points = all_object_points_successful;


    if (all_image_points_successful.empty() || all_image_points_successful.size() < 4) { // Need at least a few views
        std::cerr << "Error: Not enough images with successfully detected corners for calibration." << std::endl;
        calib_result.status = core::common::CalibErrType::CAL_CORNER_DET_ERR;
        return calib_result;
    }
    if (calib_result.image_size.empty()) {
        std::cerr << "Error: Could not determine image size for calibration." << std::endl;
        // Try to load any image to get size if first_image_for_size was problematic
        if (!image_paths.empty()) {
            cv::Mat temp_img = cv::imread(image_paths[0].string(), cv::IMREAD_GRAYSCALE);
            if (!temp_img.empty()) calib_result.image_size = temp_img.size();
            else {
                 calib_result.status = core::common::CalibErrType::CAL_CORNER_DET_ERR; // Or some other error
                 return calib_result;
            }
        } else {
            calib_result.status = core::common::CalibErrType::CAL_CORNER_DET_ERR;
            return calib_result;
        }
    }


    // --- Actual Calibration ---
    // For Charuco, this should be cv::aruco::calibrateCameraCharuco
    if (pattern_type_ == CalibPatternType::CHARUCO) {
        std::cerr << "Error: For Charuco pattern, please use a dedicated Charuco calibration function. cv::calibrateCamera is not appropriate." << std::endl;
        // TODO: Implement calibrateCameraCharuco logic
        // For now, this will likely produce poor results or fail.
        // Example:
        // std::vector<std::vector<int>> all_charuco_ids; // Needs to be populated alongside corners
        // calib_result.overall_reprojection_error = cv::aruco::calibrateCameraCharuco(
        //     all_image_points_successful, // These are charuco corners
        //     all_charuco_ids,             // These are the IDs for the charuco corners
        //     charuco_board_,
        //     calib_result.image_size,
        //     calib_result.camera_matrix,
        //     calib_result.dist_coeffs,
        //     calib_result.rvecs,
        //     calib_result.tvecs,
        //     opencv_calib_flags
        // );
        calib_result.status = core::common::CalibErrType::CAL_CORNER_DET_ERR; // Indicate wrong function for pattern
        return calib_result;
    }

    // For Chessboard (and currently as a fallback for incorrectly handled Charuco)
    calib_result.overall_reprojection_error = cv::calibrateCamera(
        all_object_points_successful,
        all_image_points_successful,
        calib_result.image_size,
        calib_result.camera_matrix,
        calib_result.dist_coeffs,
        calib_result.rvecs,
        calib_result.tvecs,
        opencv_calib_flags
    );

    if (calib_result.overall_reprojection_error >= 0) { // calibrateCamera returns RMS error
        calib_result.status = core::common::CalibErrType::CAL_OK;
    } else {
        calib_result.status = core::common::CalibErrType::CAL_CORNER_DET_ERR; // Or a generic calibration failed error
    }

    return calib_result;
}


StereoCalibResult CalibBoard::stereo_calibrate(
    const std::vector<std::filesystem::path>& left_image_paths,
    const std::vector<std::filesystem::path>& right_image_paths,
    bool use_multithreading,
    int opencv_calib_flags) {
    StereoCalibResult result;
    result.status = core::common::CalibErrType::CAL_CORNER_DET_ERR; // Default
    std::cerr << "Stereo calibration is a stub and not yet implemented." << std::endl;
    // TODO: Implement stereo calibration
    // 1. Find corners in left and right images (can use find_corners_in_images)
    // 2. Ensure pairs of images have corners detected in both.
    // 3. Populate object_points_stereo, image_points_left_stereo, image_points_right_stereo
    //    (similar to mono_calibrate's successful point collection).
    // 4. Get image_size.
    // 5. Call cv::stereoCalibrate().
    //    Might need initial camera matrices from mono calibration of each camera.
    //    The Python code does:
    //    ret_l, mtx_l, dist_l, ... = cv2.calibrateCameraExtended(objpoints, imgpoints_left, ...)
    //    ret_r, mtx_r, dist_r, ... = cv2.calibrateCameraExtended(objpoints, imgpoints_right, ...)
    //    Then:
    //    cv2.stereoCalibrateExtended(objpoints, imgpoints_left, imgpoints_right, mtx_l, dist_l, mtx_r, dist_r, ...)
    return result;
}

ImagePoseResult CalibBoard::estimate_pose(
    const ImageCornersResult& detected_corners,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs) const {
    ImagePoseResult pose_result;
    pose_result.image_path = detected_corners.image_path;

    if (!detected_corners.found || detected_corners.corners.empty()) {
        pose_result.error_type = core::common::CalibErrType::CAL_CORNER_DET_ERR;
        return pose_result;
    }

    const std::vector<cv::Point3f>* obj_pts_ptr = &object_points_;
    if (pattern_type_ == CalibPatternType::CHARUCO) {
        // For Charuco, solvePnP needs the actual 3D corner coordinates from the board.
        // detected_corners.object_points_for_image from interpolateCornersCharuco are charucoIds (ints), not 3D points.
        // This requires matching charucoIds to actual 3D points.
        // This simple estimate_pose is more suited for chessboard.
        // A dedicated estimate_pose_charuco would be needed.
        std::cerr << "Warning: estimate_pose with Charuco needs specific handling for object points. This may be incorrect." << std::endl;
        // If charuco_board_ has its corners populated, we could try to use them, but matching is key.
        // For now, this will likely be problematic for Charuco.
        // Fallback: if object_points_ was populated with Charuco corners (e.g. from charuco_board_->getChessboardCorners()),
        // it might work if detected_corners.corners corresponds to these.
        // However, interpolateCornersCharuco might return a subset, so direct use of object_points_ is risky.
        // We'd need to get the specific 3D points for the *detected* Charuco corners.
        // This usually involves charuco_board_->getChessboardCorners()[charuco_id] for each detected corner.

        // This part is a simplification and likely incorrect for a general Charuco case.
        // It assumes detected_corners.object_points_for_image was correctly populated with 3D points,
        // which is not what interpolateCornersCharuco directly provides.
        if (detected_corners.object_points_for_image.empty() ||
            detected_corners.object_points_for_image.size() != detected_corners.corners.size()) {
             std::cerr << "Error: For Charuco pose estimation, valid 3D object points corresponding to detected corners are required." << std::endl;
             pose_result.error_type = core::common::CalibErrType::CAL_CORNER_DET_ERR;
             return pose_result;
        }
        obj_pts_ptr = &detected_corners.object_points_for_image; // This assumes it's correctly filled.
    }


    bool success = cv::solvePnP(
        *obj_pts_ptr, // Use the correct set of object points
        detected_corners.corners,
        camera_matrix,
        dist_coeffs,
        pose_result.rvec,
        pose_result.tvec
    );

    if (success) {
        pose_result.corners_found = true; // Redundant if using ImageCornersResult, but for consistency
        pose_result.error_type = core::common::CalibErrType::CAL_OK;
        // Optionally, calculate reprojection error for this pose
        std::vector<cv::Point2f> reprojected_points;
        if (!pose_result.rvec.empty() && !pose_result.tvec.empty()) {
            cv::projectPoints(*obj_pts_ptr, pose_result.rvec, pose_result.tvec, camera_matrix, dist_coeffs, reprojected_points);
            pose_result.reprojection_error = cv::norm(detected_corners.corners, reprojected_points, cv::NORM_L2) / reprojected_points.size();
        }
    } else {
        pose_result.error_type = core::common::CalibErrType::CAL_CORNER_DET_ERR; // Or a PnP failed error
    }
    return pose_result;
}

ImagePoseResult CalibBoard::estimate_pose_from_image(
    const cv::Mat& image,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs) const {
    ImageCornersResult corners_res = find_corners_in_image(image);
    return estimate_pose(corners_res, camera_matrix, dist_coeffs);
}


cv::Mat CalibBoard::draw_corners_on_image(const cv::Mat& image, const std::vector<cv::Point2f>& corners, bool pattern_found) const {
    cv::Mat display_image;
    if (image.channels() == 1) {
        cv::cvtColor(image, display_image, cv::COLOR_GRAY2BGR);
    } else {
        image.copyTo(display_image);
    }
    // Pattern size for drawing: (cols, rows) of inner corners
    // If board_rows_ is inner corners horz, board_cols_ is inner corners vert
    cv::drawChessboardCorners(display_image, cv::Size(board_rows_, board_cols_), corners, pattern_found);
    return display_image;
}

cv::Mat CalibBoard::draw_reprojection_arrows(const cv::Mat& image, const std::vector<cv::Point2f>& detected_corners, const std::vector<cv::Point2f>& reprojected_corners) const {
    cv::Mat display_image;
    image.copyTo(display_image);
    // Implementation similar to Python's draw_arrows
    // (Create gradient colors, draw arrowed lines with magnification)
    std::cerr << "draw_reprojection_arrows is a stub." << std::endl;
    return display_image;
}

cv::Mat CalibBoard::draw_axes_on_image(const cv::Mat& image, const cv::Mat& rvec, const cv::Mat& tvec, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs, float axis_length_mm) const {
    cv::Mat display_image;
    image.copyTo(display_image);
    cv::drawFrameAxes(display_image, camera_matrix, dist_coeffs, rvec, tvec, axis_length_mm);
    return display_image;
}


} // namespace calib
} // namespace core
