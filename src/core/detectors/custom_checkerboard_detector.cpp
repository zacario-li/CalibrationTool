#include "custom_checkerboard_detector.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp> // For debugging, remove later if not needed
#include <iostream> // For std::cerr
#include <algorithm> // For std::min, std::max, std::sort
#include <cmath>     // For CV_PI, std::cos, std::sin, std::round, std::abs
#include <numeric>   // For std::iota

// Anonymous namespace for helper functions and constants
namespace {

// Constants from Python (TPROPS, RADIUS)
const std::vector<std::pair<double, double>> TPROPS = {
    {0.0, CV_PI / 2.0}, {CV_PI / 4.0, -CV_PI / 4.0},
    {0.0, CV_PI / 4.0}, {0.0, -CV_PI / 4.0},
    {CV_PI / 4.0, CV_PI / 2.0}, {-CV_PI / 4.0, CV_PI / 2.0},
    {-3.0 * CV_PI / 8.0, 3.0 * CV_PI / 8.0}, {-CV_PI / 8.0, CV_PI / 8.0},
    {-CV_PI / 8.0, -3.0 * CV_PI / 8.0}, {CV_PI / 8.0, 3.0 * CV_PI / 8.0}
};
// Default radiuses, can be overridden by detect_checkerboard_custom if needed by winsize
// const std::vector<int> DEFAULT_RADIUSES = {6, 8, 10};


cv::Mat normalize_image_custom(const cv::Mat& img) {
    cv::Mat gray;
    if (img.channels() > 2) {
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = img.clone();
    }

    // Python: blur_size = int(np.sqrt(gray.size) / 2) -> gray.size is total number of pixels
    // This blur_size seems excessively large and might not be what was intended.
    // A typical blur_size for a large averaging kernel might be related to a fraction of image dimension, not sqrt of total pixels.
    // Let's use a more conventional large blur for mean calculation, e.g., 1/4 of min dimension, ensuring it's odd.
    // int min_dim = std::min(gray.rows, gray.cols);
    // int blur_size = std::max(1, (min_dim / 4) | 1); // Ensure odd and at least 1
    // Python's cv2.blur(grayb, (blur_size, blur_size))
    // The Python code uses a very large blur_size for `gray_mean` (e.g., for 640x480, sqrt(307200)/2 = 277).
    // This effectively makes `gray_mean` a very smoothed version of `grayb`.
    // Let's replicate this behavior first, then consider if it needs adjustment.

    cv::Mat gray_float;
    gray.convertTo(gray_float, CV_32F); // Convert to float for calculations

    cv::Mat grayb;
    cv::GaussianBlur(gray_float, grayb, cv::Size(3,3), 1); // Python: cv2.GaussianBlur(gray, (3,3), 1)

    int blur_py_size_val = static_cast<int>(std::sqrt(static_cast<double>(gray.rows * gray.cols)) / 2.0);
    if (blur_py_size_val < 1) blur_py_size_val = 1;
    cv::Size large_blur_kernel_size(blur_py_size_val, blur_py_size_val);

    cv::Mat gray_mean;
    cv::blur(grayb, gray_mean, large_blur_kernel_size);

    cv::Mat diff = (grayb - gray_mean) / 255.0; // Normalization factor might need adjustment based on input image depth/range

    // Clip values: np.clip(diff, -0.2, 0.2)+0.2
    cv::Mat clipped_diff;
    cv::max(diff, -0.2, diff); // Lower clip
    cv::min(diff, 0.2, diff);  // Upper clip
    clipped_diff = diff + 0.2;

    // Rescale to [0, 1]: (diff - np.min(diff)) / (np.max(diff) - np.min(diff))
    double min_val, max_val;
    cv::minMaxLoc(clipped_diff, &min_val, &max_val);

    cv::Mat normalized_output;
    if (std::abs(max_val - min_val) < 1e-9) { // Avoid division by zero if all values are the same
        normalized_output = cv::Mat::zeros(clipped_diff.size(), clipped_diff.type());
    } else {
        normalized_output = (clipped_diff - min_val) / (max_val - min_val);
    }
    return normalized_output; // Output is CV_32F, range [0, 1]
}


std::vector<cv::Mat> create_correlation_patch_custom(double angle1_rad, double angle2_rad, int radius) {
    int width = radius * 2 + 1;
    int height = radius * 2 + 1;
    std::vector<cv::Mat> templates(4);

    for (int i = 0; i < 4; ++i) {
        templates[i] = cv::Mat::zeros(height, width, CV_32F);
    }

    float mu = static_cast<float>(radius);
    float mv = static_cast<float>(radius);

    cv::Point2f n1(-std::sin(angle1_rad), std::cos(angle1_rad));
    cv::Point2f n2(-std::sin(angle2_rad), std::cos(angle2_rad));

    for (int v_idx = 0; v_idx < height; ++v_idx) {
        for (int u_idx = 0; u_idx < width; ++u_idx) {
            cv::Point2f vec(static_cast<float>(u_idx) - mu, static_cast<float>(v_idx) - mv);
            float dist = static_cast<float>(cv::norm(vec));

            if (dist <= radius) {
                float s1 = vec.dot(n1);
                float s2 = vec.dot(n2);

                if (s1 <= -0.1f && s2 <= -0.1f) {
                    templates[0].at<float>(v_idx, u_idx) = 1.0f;
                } else if (s1 >= 0.1f && s2 >= 0.1f) {
                    templates[1].at<float>(v_idx, u_idx) = 1.0f;
                } else if (s1 <= -0.1f && s2 >= 0.1f) {
                    templates[2].at<float>(v_idx, u_idx) = 1.0f;
                } else if (s1 >= 0.1f && s2 <= -0.1f) {
                    templates[3].at<float>(v_idx, u_idx) = 1.0f;
                }
            }
        }
    }

    // Normalize templates (sum to 1)
    for (int i = 0; i < 4; ++i) {
        double sum_val = cv::sum(templates[i])[0];
        if (sum_val > 1e-9) { // Avoid division by zero
            templates[i] /= sum_val;
        }
    }
    return templates;
}

// Corresponds to Python's detect_corners_template
cv::Mat detect_corners_from_templates_custom(const cv::Mat& gray_normalized_float, const std::vector<cv::Mat>& templates) {
    // gray_normalized_float should be CV_32F
    if (templates.empty() || templates.size() != 4) {
        std::cerr << "Error: Exactly 4 templates are required." << std::endl;
        return cv::Mat();
    }

    std::vector<cv::Mat> img_corners(4);
    for (int i = 0; i < 4; ++i) {
        // cv::filter2D is correlation if kernel is flipped, or use matchTemplate for NCC-like scores
        // Python's signal.convolve with mode='same'
        // For correlation (which is what's usually meant by template matching in this context),
        // flip kernel for cv::filter2D or use cv::matchTemplate.
        // Given the patch creation (quadrants), this is a direct correlation/convolution.
        // Let's use filter2D. The kernel (template) should be flipped for convolution.
        // Or, if we treat it as correlation, we don't flip. Python's convolve is convolution.
        // To match Python's signal.convolve, we need to flip the kernel for filter2D or use matchTemplate.
        // However, the Python code normalizes the templates sum to 1, making them averaging filters.
        // Let's try cv::filter2D, assuming templates are already prepared (e.g. not needing flipping if they are symmetric in a way).
        // The Python `template[i] /= np.sum(template[i])` and then `signal.convolve(gray, template[i])`
        // is a weighted local average.
        // For `cv::filter2D`, the anchor is at the kernel center by default.
        cv::Mat flipped_template;
        cv::flip(templates[i], flipped_template, -1); // Flip around both axes for convolution
        cv::filter2D(gray_normalized_float, img_corners[i], -1, flipped_template, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
    }

    cv::Mat img_corners_mu = (img_corners[0] + img_corners[1] + img_corners[2] + img_corners[3]) / 4.0;

    cv::Mat arr_part1 = img_corners[0] - img_corners_mu;
    cv::Mat arr_part2 = img_corners[1] - img_corners_mu;
    cv::Mat arr_part3 = img_corners_mu - img_corners[2]; // Note the order for min
    cv::Mat arr_part4 = img_corners_mu - img_corners[3]; // Note the order for min

    cv::Mat min_arr = cv::min(arr_part1, arr_part2);
    min_arr = cv::min(min_arr, arr_part3);
    min_arr = cv::min(min_arr, arr_part4);
    // img_corners_1 in Python corresponds to this min_arr

    // For img_corners_2 (case 2: b=white, a=black), it's min(-arr)
    // -arr_part1 = img_corners_mu - img_corners[0]
    // -arr_part2 = img_corners_mu - img_corners[1]
    // -arr_part3 = img_corners[2] - img_corners_mu
    // -arr_part4 = img_corners[3] - img_corners_mu
    cv::Mat narr_part1 = img_corners_mu - img_corners[0];
    cv::Mat narr_part2 = img_corners_mu - img_corners[1];
    cv::Mat narr_part3 = img_corners[2] - img_corners_mu;
    cv::Mat narr_part4 = img_corners[3] - img_corners_mu;

    cv::Mat min_narr = cv::min(narr_part1, narr_part2);
    min_narr = cv::min(min_narr, narr_part3);
    min_narr = cv::min(min_narr, narr_part4);
    // img_corners_2 in Python corresponds to this min_narr

    cv::Mat combined_max;
    cv::max(min_arr, min_narr, combined_max); // This is the final correlation score map

    return combined_max; // CV_32F
}


// Corresponds to Python's detect_corners
cv::Mat detect_all_corner_responses_custom(const cv::Mat& gray_normalized_float, const std::vector<int>& radiuses) {
    cv::Mat out_response_map = cv::Mat::zeros(gray_normalized_float.size(), CV_32F);

    for (const auto& angles : TPROPS) {
        for (int radius : radiuses) {
            std::vector<cv::Mat> templates = create_correlation_patch_custom(angles.first, angles.second, radius);
            cv::Mat corr_map_for_this_config = detect_corners_from_templates_custom(gray_normalized_float, templates);
            if (!corr_map_for_this_config.empty()) {
                 cv::max(corr_map_for_this_config, out_response_map, out_response_map);
            }
        }
    }
    return out_response_map;
}

// Corresponds to Python's get_corner_candidates
// corr_map is expected to be CV_32F
std::vector<core::detectors::CornerCandidate> get_corner_candidates_custom(
    const cv::Mat& corr_map, int step, double threshold) {

    std::vector<core::detectors::CornerCandidate> candidates;
    if (corr_map.empty() || corr_map.type() != CV_32F) {
        std::cerr << "Error (get_corner_candidates_custom): Correlation map is empty or not CV_32F." << std::endl;
        return candidates;
    }

    // Using a set to mimic Python's `check = set()` for uniqueness of max location within overlapping regions.
    // Store as pair of ints for Point coordinates.
    std::set<std::pair<int, int>> checked_max_coords;

    for (int i = 0; i < corr_map.rows; i += step / 2) {
        for (int j = 0; j < corr_map.cols; j += step / 2) {
            cv::Rect region_rect = cv::Rect(j, i, step, step) & cv::Rect(0, 0, corr_map.cols, corr_map.rows);
            if (region_rect.area() == 0) continue;

            cv::Mat region = corr_map(region_rect);

            double min_val, max_val;
            cv::Point min_loc, max_loc;
            cv::minMaxLoc(region, &min_val, &max_val, &min_loc, &max_loc);

            if (max_val > threshold) {
                cv::Point global_max_loc(max_loc.x + region_rect.x, max_loc.y + region_rect.y);

                if (checked_max_coords.find({global_max_loc.y, global_max_loc.x}) == checked_max_coords.end()) {
                    candidates.push_back({{static_cast<float>(global_max_loc.x), static_cast<float>(global_max_loc.y)}, static_cast<float>(max_val)});
                    checked_max_coords.insert({global_max_loc.y, global_max_loc.x});
                }
            }
        }
    }
    return candidates;
}

// Corresponds to Python's non_maximum_suppression
// Using brute-force pairwise distance check instead of k-d tree for simplicity initially.
std::vector<core::detectors::CornerCandidate> non_maximum_suppression_custom(
    std::vector<core::detectors::CornerCandidate>& candidates, // Pass by value to sort, or pass by ref and expect sorted
    float min_dist_sq) { // Use squared distance to avoid sqrt

    if (candidates.empty()) return {};

    // Sort candidates by score in descending order (highest score first)
    std::sort(candidates.rbegin(), candidates.rend()); // rbegin/rend for descending

    std::vector<core::detectors::CornerCandidate> good_candidates;
    std::vector<bool> is_suppressed(candidates.size(), false);

    for (size_t i = 0; i < candidates.size(); ++i) {
        if (is_suppressed[i]) {
            continue;
        }
        good_candidates.push_back(candidates[i]); // This candidate is good

        // Suppress other candidates too close to this one
        for (size_t j = i + 1; j < candidates.size(); ++j) {
            if (is_suppressed[j]) {
                continue;
            }
            float dx = candidates[i].point.x - candidates[j].point.x;
            float dy = candidates[i].point.y - candidates[j].point.y;
            if ((dx * dx + dy * dy) < min_dist_sq) {
                is_suppressed[j] = true;
            }
        }
    }
    return good_candidates;
}

// Helper for refine_corners_custom. Solves for sub-pixel corner location in a patch.
// Based on Python's solve_patch_corner.
// patch_dx, patch_dy are CV_64F Sobel derivative patches.
cv::Point2d solve_patch_corner_custom(const cv::Mat& patch_dx, const cv::Mat& patch_dy) {
    if (patch_dx.empty() || patch_dy.empty() || patch_dx.size() != patch_dy.size() ||
        patch_dx.type() != CV_64F || patch_dy.type() != CV_64F) {
        // std::cerr << "Error (solve_patch_corner_custom): Invalid input derivative patches." << std::endl;
        return cv::Point2d(-1, -1); // Indicate error
    }

    cv::Mat mat_sum = cv::Mat::zeros(2, 2, CV_64F);
    cv::Vec2d point_sum(0.0, 0.0);

    for (int r = 0; r < patch_dx.rows; ++r) {
        for (int c = 0; c < patch_dx.cols; ++c) {
            double dx_val = patch_dx.at<double>(r, c);
            double dy_val = patch_dy.at<double>(r, c);

            // Python: vec = [dy[i,j], dx[i,j]]
            // Python: mat = np.outer(vec, vec)
            // Python: pointsum += mat.dot(pos) where pos = (i,j) -> (row, col)
            // mat = [[dy*dy, dy*dx], [dx*dy, dx*dx]]
            // mat.dot(pos) = [dy*dy*r + dy*dx*c, dx*dy*r + dx*dx*c]

            mat_sum.at<double>(0,0) += dy_val * dy_val;
            mat_sum.at<double>(0,1) += dy_val * dx_val;
            mat_sum.at<double>(1,0) += dx_val * dy_val;
            mat_sum.at<double>(1,1) += dx_val * dx_val;

            point_sum[0] += (dy_val * dy_val * r) + (dy_val * dx_val * c);
            point_sum[1] += (dx_val * dy_val * r) + (dx_val * dx_val * c);
        }
    }

    cv::Mat mat_sum_inv;
    if (cv::determinant(mat_sum) < 1e-9) { // Check for singularity
        // std::cerr << "Warning (solve_patch_corner_custom): Matrix sum is singular." << std::endl;
        return cv::Point2d(-1, -1); // Indicate error
    }
    cv::invert(mat_sum, mat_sum_inv);

    cv::Mat new_point_mat = mat_sum_inv * cv::Mat(point_sum, false); // point_sum to Mat
    return cv::Point2d(new_point_mat.at<double>(1,0), new_point_mat.at<double>(0,0)); // (col_offset, row_offset)
}


// Corresponds to Python's refine_corners
// gray_normalized_float is expected to be CV_32F
std::vector<core::detectors::CornerCandidate> refine_corners_custom(
    const std::vector<core::detectors::CornerCandidate>& candidates,
    const cv::Mat& gray_normalized_float, // Should be CV_32F
    int winsize,
    bool check_only = false) { // check_only not fully implemented as in Python, always refines if possible

    std::vector<core::detectors::CornerCandidate> refined_candidates;
    if (gray_normalized_float.empty() || gray_normalized_float.type() != CV_32F) {
        std::cerr << "Error (refine_corners_custom): Normalized gray image is invalid." << std::endl;
        return candidates; // Return original if error
    }

    int half_win = (winsize - 1) / 2;
    if (half_win <= 0) {
        std::cerr << "Error (refine_corners_custom): Winsize is too small." << std::endl;
        return candidates;
    }

    cv::Mat sobel_dx, sobel_dy;
    // Use CV_64F for Sobel derivatives for precision in solve_patch_corner_custom
    cv::Sobel(gray_normalized_float, sobel_dx, CV_64F, 1, 0, 3);
    cv::Sobel(gray_normalized_float, sobel_dy, CV_64F, 0, 1, 3);

    for (const auto& cand : candidates) {
        int y = static_cast<int>(std::round(cand.point.y));
        int x = static_cast<int>(std::round(cand.point.x));

        // Ensure window is within image bounds
        if (y - half_win < 0 || y + half_win + 1 > sobel_dx.rows ||
            x - half_win < 0 || x + half_win + 1 > sobel_dx.cols) {
            // refined_candidates.push_back(cand); // Keep original if out of bounds
            continue; // Skip if window is out of bounds, as Python version does
        }

        cv::Mat patch_dx = sobel_dx(cv::Rect(x - half_win, y - half_win, winsize, winsize));
        cv::Mat patch_dy = sobel_dy(cv::Rect(x - half_win, y - half_win, winsize, winsize));

        cv::Point2d subpixel_offset = solve_patch_corner_custom(patch_dx, patch_dy);

        if (subpixel_offset.x == -1 && subpixel_offset.y == -1) { // Error from solve_patch_corner
            // refined_candidates.push_back(cand); // Keep original if subpixel refinement failed
            continue; // Skip as in Python
        }

        // Python: newp = newp - [halfwin, halfwin]
        // newp from solve_patch_corner is already relative to patch origin (0,0)
        // So, newp is the subpixel coordinate *within* the patch.
        // We need to adjust it to be relative to the patch center (halfwin, halfwin)
        // to get the offset from the original integer candidate point.
        cv::Point2d offset_from_center(subpixel_offset.x - half_win, subpixel_offset.y - half_win);


        // Python: if np.any(np.abs(newp) > halfwin+1): continue (newp is already offset from center here)
        if (std::abs(offset_from_center.x) > half_win + 1 || std::abs(offset_from_center.y) > half_win + 1) {
            continue; // Skip if refinement is too far, as in Python
        }

        if (check_only) { // Python's check_only seems to just keep the original integer coord if valid
             refined_candidates.push_back({{static_cast<float>(x), static_cast<float>(y)}, cand.score});
        } else {
             refined_candidates.push_back({{static_cast<float>(x + offset_from_center.x),
                                            static_cast<float>(y + offset_from_center.y)},
                                            cand.score});
        }
    }
    return refined_candidates;
}

// Corresponds to Python's checkerboard_score
// corners_vec should be the reordered corners for a specific board_size.
double checkerboard_score_custom(const std::vector<cv::Point2f>& corners_vec, const cv::Size& board_size) {
    if (corners_vec.size() != static_cast<size_t>(board_size.width * board_size.height)) {
        // std::cerr << "Warning (checkerboard_score_custom): Incorrect number of corners for board size." << std::endl;
        return 1.0; // Return high score (bad)
    }

    // Reshape flat vector to 2D grid (rows = board_size.height, cols = board_size.width)
    // The order in corners_vec must match row-major or column-major as expected.
    // Python's corners_reshaped = corners[:, :2].reshape(*size, 2), size is (rows,cols) from Python perspective
    // which is (board_size.height, board_size.width) for cv::Size.
    // A point (col, row) in cv::Mat access is (x,y)
    // If corners_vec is filled row by row:
    // corner at (img_row, img_col) = corners_vec[img_row * board_size.width + img_col]

    auto get_corner = [&](int r, int c) { // r = row_idx, c = col_idx
        return corners_vec[r * board_size.width + c];
    };

    double max_metric = 0.0;

    // Check rows for collinearity issues
    for (int r_idx = 0; r_idx < board_size.height; ++r_idx) {
        for (int c_idx = 1; c_idx < board_size.width - 1; ++c_idx) {
            cv::Point2f p_prev = get_corner(r_idx, c_idx - 1);
            cv::Point2f p_curr = get_corner(r_idx, c_idx);
            cv::Point2f p_next = get_corner(r_idx, c_idx + 1);

            // top = || (p_next + p_prev) - 2*p_curr ||
            // bot = || p_next - p_prev ||
            cv::Point2f term_p_next_plus_p_prev = p_next + p_prev;
            cv::Point2f term_2_p_curr = 2.0f * p_curr;
            double top = cv::norm(term_p_next_plus_p_prev - term_2_p_curr);
            double bot = cv::norm(p_next - p_prev);

            if (std::abs(bot) < 1e-9) return 1.0; // Degenerate case, high score
            max_metric = std::max(max_metric, top / bot);
        }
    }

    // Check columns for collinearity issues
    for (int c_idx = 0; c_idx < board_size.width; ++c_idx) {
        for (int r_idx = 1; r_idx < board_size.height - 1; ++r_idx) {
            cv::Point2f p_prev = get_corner(r_idx - 1, c_idx);
            cv::Point2f p_curr = get_corner(r_idx, c_idx);
            cv::Point2f p_next = get_corner(r_idx + 1, c_idx);

            cv::Point2f term_p_next_plus_p_prev = p_next + p_prev;
            cv::Point2f term_2_p_curr = 2.0f * p_curr;
            double top = cv::norm(term_p_next_plus_p_prev - term_2_p_curr);
            double bot = cv::norm(p_next - p_prev);

            if (std::abs(bot) < 1e-9) return 1.0; // Degenerate case, high score
            max_metric = std::max(max_metric, top / bot);
        }
    }
    return max_metric; // Lower is better
}

// Corresponds to Python's make_mask_line
// Creates a binary mask with a line drawn on it.
cv::Mat make_mask_line_custom(const cv::Size& mask_shape, cv::Point2f p1, cv::Point2f p2, int thickness) {
    cv::Mat mask = cv::Mat::zeros(mask_shape, CV_8UC1);
    cv::line(mask,
             cv::Point(static_cast<int>(std::round(p1.x)), static_cast<int>(std::round(p1.y))),
             cv::Point(static_cast<int>(std::round(p2.x)), static_cast<int>(std::round(p2.y))),
             cv::Scalar(255), // Draw with white (255 for CV_8U)
             thickness);
    return mask;
}


// Corresponds to Python's reorder_checkerboard
// This is a complex function. FLANN will be used for k-NN searches.
// candidates: input corner candidates (should be refined)
// normalized_gray: CV_32F normalized grayscale image for gradient calculations
// target_board_size: cv::Size(cols, rows) of inner corners (Python size was (rows,cols))
// ordered_corners: output vector of cv::Point2f, filled if successful
// max_reorder_dist: output, maximum distance from query point to its k-NN found (Python's 'd_best')
// Returns true if successful, false otherwise.
bool reorder_checkerboard_custom(
    const std::vector<core::detectors::CornerCandidate>& candidates,
    const cv::Mat& normalized_gray_float, // CV_32F
    const cv::Size& target_board_size, // (inner_cols, inner_rows)
    std::vector<cv::Point2f>& ordered_corners,
    double& out_max_reorder_dist) {

    ordered_corners.clear();
    out_max_reorder_dist = std::numeric_limits<double>::infinity();

    if (candidates.empty() || normalized_gray_float.empty() || normalized_gray_float.type() != CV_32F) {
        std::cerr << "Error (reorder_checkerboard_custom): Invalid input." << std::endl;
        return false;
    }

    size_t num_total_corners_expected = static_cast<size_t>(target_board_size.width * target_board_size.height);
    if (candidates.size() < num_total_corners_expected) {
         std::cerr << "Warning (reorder_checkerboard_custom): Not enough candidates (" << candidates.size()
                   << ") for board size (" << num_total_corners_expected << ")." << std::endl;
        return false;
    }


    std::vector<cv::Point2f> candidate_points;
    for(const auto& cand : candidates) {
        candidate_points.push_back(cand.point);
    }
    cv::Mat candidate_points_mat(candidate_points, true); // Make a matrix from vector, true = copy data
    candidate_points_mat = candidate_points_mat.reshape(1); // Reshape to Nx2, 1 channel (CV_32FC2) -> for FLANN (CV_32F)

    if (candidate_points_mat.cols != 2) { // Should be Nx2
         candidate_points_mat = candidate_points_mat.t(); // if it's 2xN
    }
    // Ensure it's CV_32F for FLANN
    if (candidate_points_mat.type() != CV_32F) {
        candidate_points_mat.convertTo(candidate_points_mat, CV_32F);
    }


    // Build FLANN index for k-NN search
    cv::flann::Index flann_index(candidate_points_mat, cv::flann::KDTreeIndexParams());

    // Find median point among candidates (similar to Python's tree.query(np.median(...)))
    cv::Scalar mean_val = cv::mean(candidate_points_mat);
    cv::Point2f median_ish_point(static_cast<float>(mean_val[0]), static_cast<float>(mean_val[1]));

    std::vector<float> query_median = {median_ish_point.x, median_ish_point.y};
    std::vector<int> indices_mid(1);
    std::vector<float> dists_mid(1);
    flann_index.knnSearch(query_median, indices_mid, dists_mid, 1);
    int ix_mid = indices_mid[0];
    cv::Point2f corner_mid = candidate_points[ix_mid];

    // Query k=7 nearest neighbors to corner_mid (1 is self, so 6 others)
    std::vector<int> indices_knn(7);
    std::vector<float> dists_knn(7);
    flann_index.knnSearch(std::vector<float>{corner_mid.x, corner_mid.y}, indices_knn, dists_knn, 7);

    cv::Mat sobel_dx, sobel_dy, dmag;
    cv::Sobel(normalized_gray_float, sobel_dx, CV_32F, 1, 0, 3);
    cv::Sobel(normalized_gray_float, sobel_dy, CV_32F, 0, 1, 3);
    cv::magnitude(sobel_dx, sobel_dy, dmag); // CV_32F

    std::vector<cv::Point2f> selected_neighbors;
    std::vector<float> neighbor_mags;

    for (size_t k_idx = 1; k_idx < indices_knn.size(); ++k_idx) { // Skip first one (self)
        int current_neighbor_idx = indices_knn[k_idx];
        if (current_neighbor_idx >= static_cast<int>(candidate_points.size())) continue;

        cv::Point2f neighbor_pt = candidate_points[current_neighbor_idx];
        cv::Mat line_mask = make_mask_line_custom(dmag.size(), corner_mid, neighbor_pt, 3); // CV_8U mask

        cv::Mat line_mask_float;
        line_mask.convertTo(line_mask_float, CV_32F, 1.0/255.0); // Normalize mask to [0,1]

        double sum_mask = cv::sum(line_mask_float)[0];
        if (sum_mask < 1e-6) continue;
        line_mask_float /= sum_mask; // Normalize mask to sum to 1

        double mag_along_line = cv::sum(line_mask_float.mul(dmag))[0]; // Element-wise product then sum

        selected_neighbors.push_back(neighbor_pt);
        neighbor_mags.push_back(static_cast<float>(mag_along_line));
    }

    if (selected_neighbors.empty() || neighbor_mags.empty()) {
        std::cerr << "Warning (reorder_checkerboard_custom): No valid neighbors found after magnitude check." << std::endl;
        return false;
    }

    // Normalize mags (Python: mags = np.array(mags) / np.max(mags))
    float max_mag = *std::max_element(neighbor_mags.begin(), neighbor_mags.end());
    if (max_mag > 1e-6) {
        for(float& mag : neighbor_mags) mag /= max_mag;
    }

    std::vector<cv::Point2f> dirs;
    std::vector<cv::Point2f> final_selected_neighbors;

    for(size_t i=0; i<selected_neighbors.size(); ++i) {
        if (neighbor_mags[i] > 0.7f) { // Python: mags > 0.7
            dirs.push_back(selected_neighbors[i] - corner_mid);
            final_selected_neighbors.push_back(selected_neighbors[i]);
        }
    }

    if (dirs.empty() || dirs.size() < 2) {
         std::cerr << "Warning (reorder_checkerboard_custom): Not enough directions found after mag threshold." << std::endl;
        return false;
    }

    // Find ax1 (dir with max mag) and ax2 (dir most orthogonal to ax1)
    int max_mag_idx = static_cast<int>(std::distance(neighbor_mags.begin(), std::max_element(neighbor_mags.begin(), neighbor_mags.end())));
     // Ensure max_mag_idx is valid for `dirs` if `dirs` was filtered from `selected_neighbors` based on `neighbor_mags > 0.7`
    // This requires re-finding max_mag_idx within the `dirs` context or ensuring `dirs` corresponds 1:1 to `neighbor_mags > 0.7`
    // For simplicity, let's re-evaluate max_mag_idx based on `final_selected_neighbors` and their original mags.
    // This part is tricky. The Python code does `dirs = corners_selected - corner_mid` where `corners_selected` are already filtered by mag.
    // Then `ax1 = dirs[np.argmax(mags)]` where `mags` is also filtered.
    // So, if `dirs` contains points from `final_selected_neighbors`, `mags` should also be filtered.

    // Let's re-evaluate based on `dirs`. Need their original magnitudes.
    // This logic needs to be careful. The Python code is a bit dense here.
    // Simpler: find ax1 from `dirs` corresponding to the overall max magnitude neighbor.
    // This is complex because indices change. Assume `dirs` are the vectors from `corner_mid` to `final_selected_neighbors`.

    if (final_selected_neighbors.empty()) return false; // Should have been caught by dirs.empty()

    cv::Point2f ax1_vec;
    float current_max_mag_for_ax1 = -1.0f;
    // Find which of the `final_selected_neighbors` had the original highest magnitude.
    for(const auto& neighbor_pt : final_selected_neighbors) {
        for(size_t i=0; i<selected_neighbors.size(); ++i) { // Iterate original selected_neighbors
            if (cv::norm(neighbor_pt - selected_neighbors[i]) < 1e-3) { // Found the match
                if (neighbor_mags[i] > current_max_mag_for_ax1) {
                    current_max_mag_for_ax1 = neighbor_mags[i];
                    ax1_vec = neighbor_pt - corner_mid;
                }
                break;
            }
        }
    }
     if (cv::norm(ax1_vec) < 1e-6) {
        std::cerr << "Warning (reorder_checkerboard_custom): ax1_vec is near zero." << std::endl;
        return false;
    }


    cv::Point2f ax2_vec;
    float min_abs_dot_product = std::numeric_limits<float>::max();

    for(const auto& dir_vec : dirs) {
        if (cv::norm(dir_vec - ax1_vec) < 1e-3) continue; // Skip ax1 itself
        cv::Point2f dir_vec_norm = dir_vec * (1.0f / static_cast<float>(cv::norm(dir_vec)));
        cv::Point2f ax1_vec_norm = ax1_vec * (1.0f / static_cast<float>(cv::norm(ax1_vec)));
        float abs_dot = std::abs(dir_vec_norm.dot(ax1_vec_norm));
        if (abs_dot < min_abs_dot_product) {
            min_abs_dot_product = abs_dot;
            ax2_vec = dir_vec;
        }
    }
    if (cv::norm(ax2_vec) < 1e-6) {
        std::cerr << "Warning (reorder_checkerboard_custom): ax2_vec is near zero or couldn't be found." << std::endl;
        return false;
    }

    // Python: ax1 *= np.sign(np.sum(ax1))
    if ((ax1_vec.x + ax1_vec.y) < 0) ax1_vec *= -1.0f;
    if ((ax2_vec.x + ax2_vec.y) < 0) ax2_vec *= -1.0f;


    // Python: starts = np.argsort(np.dot(corners_xy, ax1 + ax2))
    // Find a few candidate starting corners.
    std::vector<std::pair<double, int>> start_candidates_scores;
    cv::Point2f combined_ax = ax1_vec + ax2_vec;
    for(size_t i=0; i<candidate_points.size(); ++i) {
        start_candidates_scores.push_back({candidate_points[i].dot(combined_ax), static_cast<int>(i)});
    }
    std::sort(start_candidates_scores.begin(), start_candidates_scores.end());

    std::vector<cv::Point2f> best_ixs_points;
    double d_best = std::numeric_limits<double>::infinity();
    // int start_best_idx = -1; // Not used in C++ version directly like this

    // Iterate through a few potential start corners (Python: starts[:2])
    int num_start_attempts = std::min(2, static_cast<int>(start_candidates_scores.size()));

    for (int attempt = 0; attempt < num_start_attempts; ++attempt) {
        int start_idx = start_candidates_scores[attempt].second;
        cv::Point2f start_xy = candidate_points[start_idx];

        for (int axis_perm = 0; axis_perm < 2; ++axis_perm) {
            cv::Point2f current_ax1 = (axis_perm == 0) ? ax1_vec : ax2_vec;
            cv::Point2f current_ax2 = (axis_perm == 0) ? ax2_vec : ax1_vec;

            // Python: _, right_ix = tree.query(ax1_test + start_xy)
            // Find nearest candidate to (start_xy + current_ax1)
            std::vector<float> query_pt_ax1 = {start_xy.x + current_ax1.x, start_xy.y + current_ax1.y};
            std::vector<int> indices_ax1_neighbor(1);
            std::vector<float> dists_ax1_neighbor(1);
            flann_index.knnSearch(query_pt_ax1, indices_ax1_neighbor, dists_ax1_neighbor, 1);

            std::vector<float> query_pt_ax2 = {start_xy.x + current_ax2.x, start_xy.y + current_ax2.y};
            std::vector<int> indices_ax2_neighbor(1);
            std::vector<float> dists_ax2_neighbor(1);
            flann_index.knnSearch(query_pt_ax2, indices_ax2_neighbor, dists_ax2_neighbor, 1);

            cv::Point2f ax1_new = 0.6f * current_ax1 + 0.4f * (candidate_points[indices_ax1_neighbor[0]] - start_xy);
            cv::Point2f ax2_new = 0.6f * current_ax2 + 0.4f * (candidate_points[indices_ax2_neighbor[0]] - start_xy);

            std::vector<cv::Point2f> current_grid_points;
            current_grid_points.reserve(num_total_corners_expected);
            std::vector<float> current_grid_query_points_flat; // For FLANN
            current_grid_query_points_flat.reserve(num_total_corners_expected * 2);

            for (int r = 0; r < target_board_size.height; ++r) { // Python size[0] is rows
                for (int c = 0; c < target_board_size.width; ++c) { // Python size[1] is cols
                    cv::Point2f query_pt = start_xy + static_cast<float>(r) * ax2_new + static_cast<float>(c) * ax1_new; // Python order xs,ys with ax1,ax2
                                                                                             // Here, r is like ys, c is like xs
                                                                                             // Python: xs[:,:,None]*ax1_new + ys[:,:,None]*ax2_new
                                                                                             // So, c maps to "xs" (along ax1_new), r maps to "ys" (along ax2_new)
                    current_grid_query_points_flat.push_back(query_pt.x);
                    current_grid_query_points_flat.push_back(query_pt.y);
                }
            }

            cv::Mat current_grid_query_mat(num_total_corners_expected, 2, CV_32F, current_grid_query_points_flat.data());
            std::vector<int> grid_indices(num_total_corners_expected);
            std::vector<float> grid_dists(num_total_corners_expected);
            flann_index.knnSearch(current_grid_query_mat, grid_indices, grid_dists, 1, cv::flann::SearchParams(32)); //Default search_params

            double max_dist_this_grid = 0;
            for(float d : grid_dists) max_dist_this_grid = std::max(max_dist_this_grid, static_cast<double>(d));

            if (max_dist_this_grid < d_best) {
                d_best = max_dist_this_grid;
                best_ixs_points.clear();
                for(int idx : grid_indices) {
                    best_ixs_points.push_back(candidate_points[idx]);
                }
                // start_best_idx = start_idx; // For debugging
            }
        }
    }

    if (best_ixs_points.size() == num_total_corners_expected) {
        ordered_corners = best_ixs_points;
        out_max_reorder_dist = d_best;
        return true;
    }

    return false;
}

// Corresponds to Python's trim_picture
// Modifies input gray image in place if trim_image is true and successful.
// Returns the top-left offset of the crop.
cv::Point trim_picture_custom(cv::Mat& gray_image) { // Input is CV_8U or CV_32F, output is same type as input
    if (gray_image.empty()) {
        std::cerr << "Error (trim_picture_custom): Input image is empty." << std::endl;
        return cv::Point(0,0);
    }
    cv::Mat gray_8u;
    if (gray_image.type() != CV_8U) {
        // Assuming input might be float, convert to 8U for Laplacian and connectedComponents
        double minVal, maxVal;
        cv::minMaxLoc(gray_image, &minVal, &maxVal);
        if (maxVal > minVal) {
            gray_image.convertTo(gray_8u, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
        } else {
            gray_image.convertTo(gray_8u, CV_8U); // Or handle error
        }
    } else {
        gray_8u = gray_image;
    }


    cv::Mat laplace, laplace_abs;
    cv::Laplacian(gray_8u, laplace, CV_64F); // Use 64F for precision
    cv::convertScaleAbs(laplace, laplace_abs); // Converts to CV_8U after taking absolute

    cv::Mat laplace_blur;
    cv::blur(laplace_abs, laplace_blur, cv::Size(100, 100)); // Large blur as in Python

    // Calculate 92nd percentile for threshold (this is computationally intensive)
    // A simpler approach like Otsu or a fixed fraction of max might be more efficient.
    // For now, let's try to mimic percentile.
    cv::Mat flat_laplace_blur = laplace_blur.reshape(1, 1); // Flatten
    std::vector<uchar> vec_laplace_blur;
    flat_laplace_blur.copyTo(vec_laplace_blur); // Copy to vector for sorting
    std::sort(vec_laplace_blur.begin(), vec_laplace_blur.end());

    double threshold_val = 0;
    if (!vec_laplace_blur.empty()) {
        threshold_val = static_cast<double>(vec_laplace_blur[static_cast<size_t>(0.92 * vec_laplace_blur.size())]);
    } else {
         std::cerr << "Warning (trim_picture_custom): vec_laplace_blur is empty." << std::endl;
         return cv::Point(0,0); // No change
    }


    cv::Mat img_thres;
    cv::threshold(laplace_blur, img_thres, threshold_val, 255, cv::THRESH_BINARY);
    img_thres.convertTo(img_thres, CV_8U); // Ensure it's 8-bit for connectedComponents

    cv::Mat labels, stats, centroids;
    int num_components = cv::connectedComponentsWithStats(img_thres, labels, stats, centroids);

    if (num_components <= 1) { // Only background component
         std::cerr << "Warning (trim_picture_custom): No components found after thresholding." << std::endl;
        return cv::Point(0,0); // No change
    }

    int best_component_idx = -1;
    int max_area = 0;
    // Start from 1 to skip background component (label 0)
    for (int i = 1; i < num_components; ++i) {
        if (stats.at<int>(i, cv::CC_STAT_AREA) > max_area) {
            max_area = stats.at<int>(i, cv::CC_STAT_AREA);
            best_component_idx = i;
        }
    }

    if (best_component_idx == -1 || max_area < 4000) { // Python: stats[best, 4] < 4000
        std::cerr << "Warning (trim_picture_custom): No suitable component found or area too small." << std::endl;
        return cv::Point(0,0); // No change
    }

    int lowx = stats.at<int>(best_component_idx, cv::CC_STAT_LEFT);
    int lowy = stats.at<int>(best_component_idx, cv::CC_STAT_TOP);
    int highx = lowx + stats.at<int>(best_component_idx, cv::CC_STAT_WIDTH);
    int highy = lowy + stats.at<int>(best_component_idx, cv::CC_STAT_HEIGHT);

    // Add padding as in Python
    lowy = std::max(0, lowy - 50);
    highy = std::min(gray_image.rows, highy + 50);
    lowx = std::max(0, lowx - 50);
    highx = std::min(gray_image.cols, highx + 50);

    if (lowy >= highy || lowx >= highx) {
        std::cerr << "Warning (trim_picture_custom): Invalid crop region after padding." << std::endl;
        return cv::Point(0,0);
    }

    gray_image = gray_image(cv::Rect(lowx, lowy, highx - lowx, highy - lowy)).clone(); // Crop and clone
    return cv::Point(lowx, lowy); // Return the offset
}


} // anonymous namespace


namespace core {
namespace detectors {

// Main function
double detect_checkerboard_custom(
    const cv::Mat& gray_image,
    const cv::Size& board_size,
    std::vector<cv::Point2f>& out_corners,
    int winsize,
    bool trim) { // Renamed from trim_image to avoid conflict with member/local

    out_corners.clear();
    if (gray_image.empty()) {
        std::cerr << "Error (detect_checkerboard_custom): Input image is empty." << std::endl;
        return 1.0; // High score for error
    }

    cv::Mat current_gray = gray_image.clone();
    cv::Point crop_offset(0, 0);

    if (trim) {
        cv::Mat original_type_gray = current_gray.clone(); // Keep original type for final crop
        crop_offset = trim_picture_custom(current_gray); // Modifies current_gray (which might be CV_8U or CV_32F)
        if (current_gray.empty()) {
            std::cerr << "Error (detect_checkerboard_custom): Image became empty after trim_picture." << std::endl;
            return 1.0;
        }
        // If trim_picture_custom changed the type (e.g. to CV_8U for processing),
        // we need to ensure current_gray for normalize_image is the cropped version of original type.
        // However, trim_picture_custom itself now handles type conversion internally and modifies input.
        // So current_gray IS the cropped image.
    }

    cv::Mat normalized_gray = normalize_image_custom(current_gray); // Expects CV_8U or CV_32F, outputs CV_32F

    std::vector<int> radiuses;
     // Python: radiuses = [winsize+3]; if winsize >= 8: radiuses.append(winsize-3)
    radiuses.push_back(winsize + 3);
    if (winsize >= 8) {
        radiuses.push_back(winsize - 3);
    }
    radiuses.erase(std::remove_if(radiuses.begin(), radiuses.end(), [](int r){ return r <= 0; }), radiuses.end());
    if (radiuses.empty()) { // Default if winsize is too small
        radiuses = {6, 8, 10}; // Matching Python's RADIUS = [6, 8, 10] default for detect_corners
    }


    cv::Mat corr_map = detect_all_corner_responses_custom(normalized_gray, radiuses);

    cv::Mat corr_blurred;
    cv::GaussianBlur(corr_map, corr_blurred, cv::Size(7,7), 3);

    double max_corr_val = 0;
    cv::minMaxLoc(corr_blurred, nullptr, &max_corr_val);

    std::vector<CornerCandidate> candidates = get_corner_candidates_custom(corr_blurred, winsize + 2, max_corr_val * 0.2);

    size_t expected_corners_count = static_cast<size_t>(board_size.width * board_size.height);
    if (candidates.size() < expected_corners_count) {
        // std::cerr << "Warning (detect_checkerboard_custom): Not enough candidates after get_corner_candidates." << std::endl;
        return 1.0;
    }

    // NMS min_dist is winsize-2 in Python code. Squared for efficiency.
    float nms_min_dist = static_cast<float>(winsize - 2);
    if (nms_min_dist < 1.0f) nms_min_dist = 1.0f; // Ensure positive distance
    candidates = non_maximum_suppression_custom(candidates, nms_min_dist * nms_min_dist);

    if (candidates.size() < expected_corners_count) {
         // std::cerr << "Warning (detect_checkerboard_custom): Not enough candidates after NMS." << std::endl;
        return 1.0;
    }

    std::vector<CornerCandidate> refined_candidates_sp = refine_corners_custom(candidates, normalized_gray, winsize + 2);
    // Python also calls refine_corners again with smaller window sizes, which are currently not implemented here.
    // corners_sp = refine_corners(corners_sp, diff, winsize=max(winsize//2-1,5), check_only=True)
    // corners_sp = refine_corners(corners_sp, diff, winsize=5, check_only=True)
    // These additional refinements with check_only=true might be important for robustness.
    // For now, using only one pass of refinement.

    // Sort by score for reorder_checkerboard, as Python's best_ix = np.argsort(-scores) implies.
    // NMS already sorted them by score descending. If refine_corners_custom preserves order or re-sorts, it's fine.
    // Assuming refine_corners_custom does not re-sort or if it does, it's by score.
    // Python takes [:num_corners+3] or [:num_corners+10] for reordering.
    // Let's take a slightly larger set than board_size for reordering if available.
    size_t num_candidates_for_reorder = std::min(candidates.size(), expected_corners_count + 10); // Python uses +3 then +10
    std::vector<CornerCandidate> candidates_for_reorder(refined_candidates_sp.begin(),
                                                        refined_candidates_sp.begin() + std::min(refined_candidates_sp.size(), num_candidates_for_reorder));


    std::vector<cv::Point2f> ordered_corners_raw;
    double max_reorder_dist = -1.0;
    bool reorder_ok = reorder_checkerboard_custom(candidates_for_reorder, normalized_gray, board_size, ordered_corners_raw, max_reorder_dist);

    if (!reorder_ok || ordered_corners_raw.size() != expected_corners_count) {
        // Attempt with more points if initial reorder failed, similar to Python's second try
        num_candidates_for_reorder = std::min(candidates.size(), expected_corners_count + 20); // Try with more points
        std::vector<CornerCandidate> more_candidates_for_reorder(refined_candidates_sp.begin(),
                                                                 refined_candidates_sp.begin() + std::min(refined_candidates_sp.size(), num_candidates_for_reorder));
        reorder_ok = reorder_checkerboard_custom(more_candidates_for_reorder, normalized_gray, board_size, ordered_corners_raw, max_reorder_dist);
        if (!reorder_ok || ordered_corners_raw.size() != expected_corners_count) {
            // std::cerr << "Warning (detect_checkerboard_custom): Reordering checkerboard failed." << std::endl;
            return 1.0;
        }
    }

    double final_score = checkerboard_score_custom(ordered_corners_raw, board_size);

    // Python conditions: if check_score > 0.3 or len(best_corners) < num_corners or max_dist > winsize*3: return None, 1.0
    // max_reorder_dist is squared in Python's d_best from knn tree, here it's direct distance.
    // The max_dist check from Python (`max_dist > winsize*3`) is not directly translated for `out_max_reorder_dist` yet.
    // `out_max_reorder_dist` is the max distance of a *projected grid point* to its *actual nearest candidate*.
    // This is different from Python's `max_dist` in `reorder_checkerboard` which is `d_best` from the grid fitting.
    // For now, mainly relying on `final_score`.

    if (final_score > 0.3) { // Threshold from Python
        // std::cerr << "Warning (detect_checkerboard_custom): Final checkerboard score is too high: " << final_score << std::endl;
        return final_score; // Return the score, indicates poor quality
    }

    // Adjust for crop_offset
    out_corners.resize(ordered_corners_raw.size());
    for(size_t i=0; i < ordered_corners_raw.size(); ++i) {
        out_corners[i] = ordered_corners_raw[i] + cv::Point2f(static_cast<float>(crop_offset.x), static_cast<float>(crop_offset.y));
    }

    return final_score; // Lower is better
}

} // namespace detectors
} // namespace core
