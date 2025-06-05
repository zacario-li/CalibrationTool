#include "transformation_utils.h"
#include <opencv2/imgproc.hpp> // Not strictly needed for these, but often included with calib3d
#include <iostream> // For std::cerr
#include <cmath>     // For acos, sqrt, etc.

namespace core {
namespace calib {

cv::Mat combine_rt_to_homogeneous(const cv::Mat& rotation_matrix, const cv::Mat& translation_vector) {
    if (rotation_matrix.cols != 3 || rotation_matrix.rows != 3) {
        std::cerr << "Error: Rotation matrix must be 3x3." << std::endl;
        return cv::Mat();
    }
    if (!((translation_vector.cols == 1 && translation_vector.rows == 3) || (translation_vector.cols == 3 && translation_vector.rows == 1))) {
        std::cerr << "Error: Translation vector must be 3x1 or 1x3." << std::endl;
        return cv::Mat();
    }

    cv::Mat homogeneous_matrix = cv::Mat::eye(4, 4, rotation_matrix.type());

    rotation_matrix.copyTo(homogeneous_matrix(cv::Rect(0, 0, 3, 3)));

    if (translation_vector.rows == 3 && translation_vector.cols == 1) { // 3x1 column vector
        translation_vector.copyTo(homogeneous_matrix(cv::Rect(3, 0, 1, 3)));
    } else { // 1x3 row vector, transpose it
        cv::Mat t_vec_col = translation_vector.t();
        t_vec_col.copyTo(homogeneous_matrix(cv::Rect(3, 0, 1, 3)));
    }

    return homogeneous_matrix;
}


cv::Vec4d rotation_matrix_to_quaternion(const cv::Mat& R) {
    if (R.cols != 3 || R.rows != 3) {
        std::cerr << "Error: Rotation matrix must be 3x3 for quaternion conversion." << std::endl;
        return cv::Vec4d(0,0,0,0); // Return invalid quaternion
    }

    cv::Vec4d q; // OpenCV uses (x, y, z, w) for some functions, but typically (w, x, y, z) is standard. We'll use (w,x,y,z)
    double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);

    if (trace > 0.0) {
        double s = 0.5 / std::sqrt(trace + 1.0);
        q[0] = 0.25 / s; // w
        q[1] = (R.at<double>(2,1) - R.at<double>(1,2)) * s; // x
        q[2] = (R.at<double>(0,2) - R.at<double>(2,0)) * s; // y
        q[3] = (R.at<double>(1,0) - R.at<double>(0,1)) * s; // z
    } else {
        if (R.at<double>(0,0) > R.at<double>(1,1) && R.at<double>(0,0) > R.at<double>(2,2)) {
            double s = 2.0 * std::sqrt(1.0 + R.at<double>(0,0) - R.at<double>(1,1) - R.at<double>(2,2));
            q[0] = (R.at<double>(2,1) - R.at<double>(1,2)) / s; // w
            q[1] = 0.25 * s; // x
            q[2] = (R.at<double>(0,1) + R.at<double>(1,0)) / s; // y
            q[3] = (R.at<double>(0,2) + R.at<double>(2,0)) / s; // z
        } else if (R.at<double>(1,1) > R.at<double>(2,2)) {
            double s = 2.0 * std::sqrt(1.0 + R.at<double>(1,1) - R.at<double>(0,0) - R.at<double>(2,2));
            q[0] = (R.at<double>(0,2) - R.at<double>(2,0)) / s; // w
            q[1] = (R.at<double>(0,1) + R.at<double>(1,0)) / s; // x
            q[2] = 0.25 * s; // y
            q[3] = (R.at<double>(1,2) + R.at<double>(2,1)) / s; // z
        } else {
            double s = 2.0 * std::sqrt(1.0 + R.at<double>(2,2) - R.at<double>(0,0) - R.at<double>(1,1));
            q[0] = (R.at<double>(1,0) - R.at<double>(0,1)) / s; // w
            q[1] = (R.at<double>(0,2) + R.at<double>(2,0)) / s; // x
            q[2] = (R.at<double>(1,2) + R.at<double>(2,1)) / s; // y
            q[3] = 0.25 * s; // z
        }
    }
    // Normalize the quaternion
    double norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 1e-9) { // Avoid division by zero
       for(int i=0; i<4; ++i) q[i] /= norm;
    }
    return q;
}

cv::Mat quaternion_to_rotation_matrix(const cv::Vec4d& q_in) {
    // Ensure quaternion is normalized (w, x, y, z)
    double norm = cv::norm(q_in);
    if (norm < 1e-9) { // Prevent division by zero for zero quaternion
        std::cerr << "Warning: Zero quaternion provided to quaternion_to_rotation_matrix. Returning identity matrix." << std::endl;
        return cv::Mat::eye(3, 3, CV_64F);
    }
    cv::Vec4d q = q_in / norm;


    double w = q[0];
    double x = q[1];
    double y = q[2];
    double z = q[3];

    cv::Mat R(3, 3, CV_64F);
    R.at<double>(0,0) = 1 - 2*y*y - 2*z*z;
    R.at<double>(0,1) = 2*x*y - 2*w*z;
    R.at<double>(0,2) = 2*x*z + 2*w*y;

    R.at<double>(1,0) = 2*x*y + 2*w*z;
    R.at<double>(1,1) = 1 - 2*x*x - 2*z*z;
    R.at<double>(1,2) = 2*y*z - 2*w*x;

    R.at<double>(2,0) = 2*x*z - 2*w*y;
    R.at<double>(2,1) = 2*y*z + 2*w*x;
    R.at<double>(2,2) = 1 - 2*x*x - 2*y*y;

    return R;
}

double compute_rotation_angle_delta(const cv::Mat& r_initial, const cv::Mat& r_final) {
    if (r_initial.cols != 3 || r_initial.rows != 3 || r_final.cols != 3 || r_final.rows != 3) {
        std::cerr << "Error: Rotation matrices must be 3x3." << std::endl;
        return -1.0; // Error value
    }
    if (cv::determinant(r_initial) == 0) {
         std::cerr << "Error: Initial rotation matrix is singular." << std::endl;
        return -1.0;
    }

    cv::Mat delta_R = r_final * r_initial.inv();
    double trace_delta_R = cv::trace(delta_R);

    // Clamp value to avoid domain errors with acos due to floating point inaccuracies
    double cos_theta = (trace_delta_R - 1.0) / 2.0;
    if (cos_theta > 1.0) {
        cos_theta = 1.0;
    } else if (cos_theta < -1.0) {
        cos_theta = -1.0;
    }

    double theta_rad = std::acos(cos_theta);
    return theta_rad * (180.0 / CV_PI); // Convert to degrees
}


} // namespace calib
} // namespace core
