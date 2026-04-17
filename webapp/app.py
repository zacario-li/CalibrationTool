"""
Flask Web Application - Camera Calibration Tool

Features:
- Mono camera calibration with checkerboard detection
- Stereo camera calibration and rectification
- Hand-eye calibration for robotics (AXXB)
- Stereo disparity/depth measurement

Usage:
    python app.py

Access: http://localhost:5000
"""

import sys
import os
import tempfile
import shutil
import time
import base64
from datetime import datetime
from flask import Flask, render_template, request, jsonify, make_response
from werkzeug.utils import secure_filename
from loguru import logger
from pathlib import Path
import cv2
import numpy as np
import json

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from utils.calib import CalibChessboard, HandEye, load_camera_param
from utils.ophelper import combine_RT

# Initialize Flask app
app = Flask(__name__, template_folder='templates', static_folder='static')

# Configuration
app.config['MAX_CONTENT_LENGTH'] = 500 * 1024 * 1024
app.config['SECRET_KEY'] = 'dev-secret-key-12345'
app.config['ALLOWED_EXTENSIONS'] = {'png', 'jpg', 'jpeg', 'bmp', 'webp', 'csv', 'txt', 'json'}

# Configure logging
logger.remove()
logger.add(sys.stdout, format="{level} | {time:HH:mm:ss} | {message}", level="DEBUG")


def allowed_file(filename):
    """Check if file has allowed extension."""
    if not filename or '.' not in filename:
        return False
    return filename.rsplit('.', 1)[1].lower() in app.config['ALLOWED_EXTENSIONS']


@app.context_processor
def inject_variables():
    """Add common variables to all templates."""
    return {
        'app_name': 'Camera Calibration Tool',
        'current_year': datetime.now().year
    }


@app.route('/')
def index():
    """Main dashboard page."""
    return render_template('index.html')


@app.route('/calibration/mono')
def mono_calibration():
    """Mono camera calibration page."""
    return render_template('mono_calibration.html')


@app.route('/calibration/stereo')
def stereo_calibration():
    """Stereo camera calibration page."""
    return render_template('stereo_calibration.html')


@app.route('/calibration/handeye')
def handeye_calibration():
    """Hand-eye calibration page."""
    return render_template('handeye_calibration.html')


@app.route('/calibration/disparity')
def disparity_calibration():
    """Stereo disparity computation page."""
    return render_template('disparity.html')


@app.route('/api/mono/calibrate', methods=['POST'])
def mono_calibrate():
    """
    Mono camera calibration API endpoint.

    Args:
        rows (int): Checkerboard rows (inner points)
        cols (int): Checkerboard columns (inner points)
        cellsize (float): Cell size in mm
        use_libcbdet (bool): Whether to use libcbdetect
        files (list): List of calibration images

    Returns:
        JSON response with calibration results
    """
    try:
        logger.info("=" * 50)
        logger.info("Mono camera calibration request received")

        rows = int(request.form.get('rows', 9))
        cols = int(request.form.get('cols', 6))
        cell_size = float(request.form.get('cellsize', 1.0))
        use_libcbdet = request.form.get('use_libcbdet', 'false') == 'true'
        files = request.files.getlist('files')

        logger.info(f"Parameters: rows={rows}, cols={cols}, cell_size={cell_size}mm")
        logger.info(f"Uploaded files: {len(files)} images")
        logger.info(f"Using libcbdetect: {use_libcbdet}")

        if not files or len(files) == 0:
            return jsonify({'success': False, 'error': 'No images uploaded'}), 400

        if not all(allowed_file(f.filename) for f in files):
            return jsonify({'success': False, 'error': 'Invalid file format'}), 400

        temp_dir = tempfile.mkdtemp()
        image_files = []
        timestamp = datetime.now().strftime("%Y%m%d%H%M%S%f")

        try:
            for i, file in enumerate(files):
                filename = secure_filename(f"{timestamp}_{i:03d}_{file.filename}")
                filepath = os.path.join(temp_dir, filename)
                file.save(filepath)
                image_files.append(filepath)
                logger.debug(f"Saved: {filename}")

            filenames = [os.path.basename(f) for f in image_files]
            calib = CalibChessboard(rows, cols, cell_size, use_mt=True, use_libcbdet=use_libcbdet)

            logger.info(f"Processing {len(filenames)} images for calibration")

            tic = time.time()
            try:
                result = calib.mono_calib(temp_dir, filenames)
                duration = time.time() - tic
                logger.info(f"Calibration completed in {duration:.2f} seconds")

                ret, mtx, dist, rvecs, tvecs, perverrs, rejected_files, calibrated_files, shape, imgpoints, error_type = result

                logger.info(f"Calibration result: ret={ret}, error={error_type}")
                logger.info(f"Images processed: {len(calibrated_files) if calibrated_files else 0}")
                logger.info(f"Images rejected: {len(rejected_files) if rejected_files else 0}")

                if ret:
                    # perverrs is an array of per-image reprojection errors (N, 1) shape
                    perverrs_list = perverrs.flatten().tolist() if perverrs is not None else []
                    mean_error = float(np.mean(perverrs)) if perverrs is not None else 0.0

                    # rvecs and tvecs are lists of arrays from calibrateCameraExtended
                    rvecs_list = [np.array(rv).tolist() if isinstance(rv, np.ndarray) else list(rv) for rv in rvecs] if rvecs else []
                    tvecs_list = [np.array(tv).tolist() if isinstance(tv, np.ndarray) else list(tv) for tv in tvecs] if tvecs else []

                    return jsonify({
                        'success': True,
                        'intrinsic': mtx.tolist() if hasattr(mtx, 'tolist') else [[float(x) for x in row] for row in mtx],
                        'distortion': dist.tolist() if hasattr(dist, 'tolist') else [[float(x) for x in row] for row in dist],
                        'reproj_error_mean': mean_error,
                        'reproj_errors': perverrs_list,
                        'rvecs': rvecs_list,
                        'tvecs': tvecs_list,
                        'shape': list(shape) if shape is not None else [0, 0],
                        'processed': len(calibrated_files) if calibrated_files else 0,
                        'rejected': len(rejected_files) if rejected_files else 0,
                        'rejected_files': rejected_files if rejected_files else []
                    })
                else:
                    return jsonify({
                        'success': False,
                        'error': f'Calibration failed: {error_type}',
                        'processed': len(calibrated_files) if calibrated_files else 0,
                        'rejected': len(rejected_files) if rejected_files else 0,
                        'rejected_files': rejected_files if rejected_files else []
                    })

            except Exception as calib_err:
                logger.exception(f"Calibration error: {calib_err}")
                return jsonify({'success': False, 'error': f'Calibration error: {str(calib_err)}'}), 500

        finally:
            try:
                shutil.rmtree(temp_dir)
                logger.info("Temporary files cleaned up")
            except Exception as e:
                logger.warning(f"Cleanup error: {e}")

    except Exception as e:
        logger.exception(f"Mono calibration exception: {e}")
        return jsonify({'success': False, 'error': f'Internal server error: {str(e)}'}), 500


@app.route('/api/stereo/calibrate', methods=['POST'])
def stereo_calibrate():
    """
    Stereo camera calibration API endpoint.

    Args:
        rows (int): Checkerboard rows
        cols (int): Checkerboard columns
        cellsize (float): Cell size in mm
        use_libcbdet (bool): Whether to use libcbdetect
        files_left (list): Left camera images
        files_right (list): Right camera images

    Returns:
        JSON response with stereo calibration results
    """
    try:
        logger.info("=" * 50)
        logger.info("Stereo camera calibration request received")

        rows = int(request.form.get('rows', 9))
        cols = int(request.form.get('cols', 6))
        cell_size = float(request.form.get('cellsize', 1.0))
        use_libcbdet = request.form.get('use_libcbdet', 'false') == 'true'

        files_left = request.files.getlist('files_left')
        files_right = request.files.getlist('files_right')

        logger.info(f"Parameters: rows={rows}, cols={cols}, cell_size={cell_size}mm")
        logger.info(f"Left images: {len(files_left)}, Right images: {len(files_right)}")

        if not files_left or not files_right:
            return jsonify({'success': False, 'error': 'No images uploaded'}), 400

        if len(files_left) != len(files_right):
            return jsonify({'success': False, 'error': 'Mismatched image count'}), 400

        temp_left = tempfile.mkdtemp()
        temp_right = tempfile.mkdtemp()
        left_files = []
        right_files = []
        timestamp = datetime.now().strftime("%Y%m%d%H%M%S%f")

        try:
            for i, file in enumerate(files_left):
                if allowed_file(file.filename):
                    path = os.path.join(temp_left, f"{timestamp}{i:04d}_L_{file.filename}")
                    file.save(path)
                    left_files.append(path)

            for i, file in enumerate(files_right):
                if allowed_file(file.filename):
                    path = os.path.join(temp_right, f"{timestamp}{i:04d}_R_{file.filename}")
                    file.save(path)
                    right_files.append(path)

            logger.info(f"Saved {len(left_files)} left and {len(right_files)} right images")

            calib = CalibChessboard(rows, cols, cell_size, use_mt=True, use_libcbdet=use_libcbdet)

            lfnames = [os.path.basename(f) for f in left_files]
            rfnames = [os.path.basename(f) for f in right_files]

            logger.info(f"Processing {len(lfnames)} stereo pairs")

            result = calib.stereo_calib(temp_left, temp_right, lfnames, rfnames)

            logger.info(f"Stereo calibration ret: {result[0]}, error: {result[17]}")

            if result[0]:
                # Get stereo reprojection error
                stereo_error = 0.0
                if len(result) > 11 and result[11] is not None:
                    try:
                        pererr = result[11]
                        if hasattr(pererr, 'mean'):
                            stereo_error = float(pererr.mean())
                    except (ValueError, TypeError, RuntimeError):
                        stereo_error = 0.0

                # Generate rectification preview from first calibrated pair
                rectified_images = []
                calibrated_files = result[13] if len(result) > 13 else []
                if len(calibrated_files) > 0:
                    try:
                        first_pair = calibrated_files[0]
                        left_path = os.path.join(temp_left, first_pair[0])
                        right_path = os.path.join(temp_right, first_pair[1])

                        left_img = cv2.imread(left_path)
                        right_img = cv2.imread(right_path)

                        if left_img is not None and right_img is not None:
                            # Compute rectification transforms
                            img_size = (left_img.shape[1], left_img.shape[0])
                            R1, R2, P1, P2, Q, validPixRoy1, validPixRoy2 = cv2.stereoRectify(
                                result[1], result[2], result[3], result[4],
                                img_size, result[5], result[6])

                            # Compute rectification maps
                            map1x, map1y = cv2.initUndistortRectifyMap(
                                result[1], result[2], R1, P1, img_size, cv2.CV_16SC2)
                            map2x, map2y = cv2.initUndistortRectifyMap(
                                result[3], result[4], R2, P2, img_size, cv2.CV_16SC2)

                            # Apply rectification
                            rectified_left = cv2.remap(left_img, map1x, map1y, cv2.INTER_LINEAR)
                            rectified_right = cv2.remap(right_img, map2x, map2y, cv2.INTER_LINEAR)

                            # Save as base64
                            _, buf_left = cv2.imencode('.png', rectified_left)
                            _, buf_right = cv2.imencode('.png', rectified_right)
                            rectified_images = [
                                f"data:image/png;base64,{base64.b64encode(buf_left).decode()}",
                                f"data:image/png;base64,{base64.b64encode(buf_right).decode()}"
                            ]
                    except Exception as e:
                        logger.warning(f"Failed to generate rectification preview: {e}")

                return jsonify({
                    'success': True,
                    'left_intrinsic': result[1].tolist(),
                    'left_distortion': result[2].tolist(),
                    'right_intrinsic': result[3].tolist(),
                    'right_distortion': result[4].tolist(),
                    'rotation': result[5].tolist(),
                    'translation': result[6].tolist(),
                    'essential': result[7].tolist(),
                    'fundamental': result[8].tolist(),
                    'processed': len(result[13]),
                    'rejected': len(result[12]),
                    'stereo_error': stereo_error,
                    'rectified_images': rectified_images
                })
            else:
                return jsonify({
                    'success': False,
                    'error': f'Stereo calibration failed: {result[17]}',
                    'processed': len(result[15]),
                    'rejected': len(result[14])
                })

        except Exception as e:
            logger.exception(f"Stereo calibration error: {e}")
            return jsonify({'success': False, 'error': str(e)}), 500

        finally:
            try:
                shutil.rmtree(temp_left, ignore_errors=True)
                shutil.rmtree(temp_right, ignore_errors=True)
            except:
                pass

    except Exception as e:
        logger.exception(f"Stereo calibration exception: {e}")
        return jsonify({'success': False, 'error': f'Internal server error: {str(e)}'}), 500


@app.route('/api/handeye/calibrate', methods=['POST'])
def handeye_calibrate():
    """
    Hand-eye calibration API endpoint.

    Workflow:
    1. Load robot motion data (gripper poses)
    2. Load camera intrinsics
    3. Detect checkerboard in images to compute camera poses
    4. Solve AX=XB hand-eye calibration

    Args:
        robot_data (file): Robot motion data (.csv or .txt)
        camera_params (file): Camera intrinsics (.json)
        cb_images (list): Checkerboard images
        cb_rows, cb_cols (int): Checkerboard squares
        cb_cellsize (float): Cell size in mm
        calib_method (str): Calibration algorithm
        use_right_camera (bool): Use right camera in stereo
        need_transpose (bool): Transpose camera matrix

    Returns:
        JSON response with hand-eye calibration results
    """
    try:
        logger.info("=" * 50)
        logger.info("Hand-eye calibration request received")

        robot_data = request.files.get('robot_data')
        camera_params = request.files.get('camera_params')
        cb_images = request.files.getlist('cb_images')

        cb_rows = int(request.form.get('cb_rows', 12))
        cb_cols = int(request.form.get('cb_cols', 9))
        cb_cellsize = float(request.form.get('cb_cellsize', 5.0))
        use_rvec = request.form.get('use_rvec', 'false') == 'true'
        rotation_only = request.form.get('rotation_only', 'false') == 'true'
        use_right_camera = request.form.get('use_right_camera', 'false') == 'true'
        need_transpose = request.form.get('need_transpose', 'true') == 'true'
        use_libcbdet = request.form.get('use_libcbdet', 'false') == 'true'
        calib_method_str = request.form.get('calib_method', '0')

        if not robot_data or not camera_params or len(cb_images) == 0:
            return jsonify({'success': False, 'error': 'Missing required files'}), 400

        if len(cb_images) < 4:
            return jsonify({'success': False, 'error': 'Need at least 4 checkerboard images'}), 400

        # Map calibration method
        calib_method_map = {
            '0': cv2.CALIB_HAND_EYE_TSAI,
            '1': cv2.CALIB_HAND_EYE_PARK,
            '2': cv2.CALIB_HAND_EYE_DANIILIDIS,
            '3': cv2.CALIB_HAND_EYE_HORAUD,
            '4': cv2.CALIB_HAND_EYE_ANDREFF
        }
        calib_method = calib_method_map.get(calib_method_str, cv2.CALIB_HAND_EYE_TSAI)

        temp_dir = tempfile.mkdtemp()
        robot_path = os.path.join(temp_dir, 'robot_data')
        camera_path = os.path.join(temp_dir, 'camera_params.json')
        images_dir = os.path.join(temp_dir, 'images')
        os.makedirs(images_dir)

        robot_data.save(robot_path)
        camera_params.save(camera_path)

        # Save checkerboard images
        image_filenames = []
        for i, img in enumerate(cb_images):
            filename = f"img_{i:03d}.png"
            img.save(os.path.join(images_dir, filename))
            image_filenames.append(filename)

        logger.info(f"Loaded {len(cb_images)} checkerboard images")

        load_trans = need_transpose
        camera_id = use_right_camera

        # Initialize calibration objects
        handeye = HandEye()
        calib_cb = CalibChessboard(cb_rows, cb_cols, cb_cellsize, use_mt=True, use_libcbdet=use_libcbdet)

        # Parse robot data (gripper poses)
        if use_rvec:
            R_g2n_list, T_g2n_list = handeye.generate_gripper2base_with_rvec_txt(robot_path)
        else:
            R_g2n_list, T_g2n_list = handeye.generate_gripper2ndi_with_file(
                robot_path, sensor_only=rotation_only, randomtest=rotation_only)

        logger.info(f"Loaded {len(R_g2n_list)} robot poses")

        if len(R_g2n_list) != len(image_filenames):
            return jsonify({
                'success': False,
                'error': f'Robot data count ({len(R_g2n_list)}) does not match image count ({len(image_filenames)})'
            }), 400

        # Load camera parameters
        intri, dist = load_camera_param(camera_path, need_trans=load_trans, camera_id=camera_id)

        # Compute camera poses from checkerboard images
        R_b2c_list = []
        t_b2c_list = []
        img_for_preview = None

        for i, fname in enumerate(image_filenames):
            img = cv2.imread(os.path.join(images_dir, fname))
            if img is None:
                continue

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            found, corners = calib_cb.find_corners(gray)

            if not found:
                logger.warning(f"Could not find checkerboard in {fname}")
                continue

            # Solve PnP for camera pose
            ret, rvec, tvec, _ = cv2.solvePnPRansac(
                calib_cb.objp, corners.reshape(-1, 2), intri, dist
            )

            if not ret:
                logger.warning(f"PnP failed for {fname}")
                continue

            R, _ = cv2.Rodrigues(rvec)
            R_b2c_list.append(R)
            t_b2c_list.append(tvec)

            if img_for_preview is None:
                img_for_preview = cv2.drawChessboardCorners(img, (cb_rows-1, cb_cols-1), corners, True)

        logger.info(f"Computed {len(R_b2c_list)} camera poses")

        if len(R_b2c_list) < 4:
            return jsonify({
                'success': False,
                'error': f'Only {len(R_b2c_list)} valid camera poses detected (need at least 4)'
            }), 400

        # Perform AX=XB hand-eye calibration
        r_c2g, t_c2g, rot_err, trans_err = handeye.calib_axxb(
            R_g2n_list[:len(R_b2c_list)], T_g2n_list[:len(R_b2c_list)],
            R_b2c_list, t_b2c_list, calib_method
        )

        # Create preview image as base64
        corners_preview = None
        if img_for_preview is not None:
            _, buffer = cv2.imencode('.png', img_for_preview)
            corners_preview = f"data:image/png;base64,{buffer.tobytes().hex()}"

        result = {
            'R_c2g': r_c2g.tolist() if hasattr(r_c2g, 'tolist') else [[float(x) for x in row] for row in r_c2g],
            't_c2g': t_c2g.tolist() if hasattr(t_c2g, 'tolist') else [float(x) for x in t_c2g.flatten()],
            'rotation_error': float(rot_err),
            'translation_error': float(trans_err),
            'valid_images': len(R_b2c_list)
        }

        return jsonify({'success': True, 'result': result, 'corners_preview': corners_preview})

    except Exception as e:
        logger.exception(f"Hand-eye calibration error: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

    finally:
        try:
            shutil.rmtree(temp_dir, ignore_errors=True)
        except:
            pass





@app.route('/api/disparity/compute', methods=['POST'])
def disparity_compute():
    """
    Stereo disparity and depth computation using SGBM algorithm.

    Args:
        stereo_params (file): Stereo calibration JSON file
        left_image (file): Left camera image
        right_image (file): Right camera image
        num_disparities (int): Number of disparities (multiple of 16)
        block_size (int): Block size for matching
        uniqueness_ratio (int): Uniqueness ratio
        speckle_window_size (int): Speckle filter window
        speckle_range (int): Speckle filter range
        disp12_max_diff (int): Max difference between left/right disparities

    Returns:
        JSON response with disparity map, depth map, and statistics
    """
    try:
        logger.info("=" * 50)
        logger.info("Disparity computation request received")

        stereo_params_file = request.files.get('stereo_params')
        left_image_file = request.files.get('left_image')
        right_image_file = request.files.get('right_image')

        num_disparities = int(request.form.get('num_disparities', 256))
        block_size = int(request.form.get('block_size', 7))
        uniqueness_ratio = int(request.form.get('uniqueness_ratio', 15))
        speckle_window_size = int(request.form.get('speckle_window_size', 100))
        speckle_range = int(request.form.get('speckle_range', 32))
        disp12_max_diff = int(request.form.get('disp12_max_diff', 3))

        # Ensure num_disparities is multiple of 16
        if num_disparities % 16 != 0:
            num_disparities = ((num_disparities // 16) + 1) * 16

        if not stereo_params_file or not left_image_file or not right_image_file:
            return jsonify({'success': False, 'error': 'Missing required files'}), 400

        temp_dir = tempfile.mkdtemp()

        # Save uploaded files
        stereo_path = os.path.join(temp_dir, 'stereo_params.json')
        left_path = os.path.join(temp_dir, 'left.png')
        right_path = os.path.join(temp_dir, 'right.png')

        stereo_params_file.save(stereo_path)
        left_image_file.save(left_path)
        right_image_file.save(right_path)

        # Load stereo calibration parameters
        with open(stereo_path, 'r') as f:
            stereo_params = json.load(f)

        # Extract camera parameters
        # Get left camera parameters
        if 'CameraParameters1' in stereo_params:
            cam1_elem = 'CameraParameters1'
        elif 'CameraParameters' in stereo_params:
            cam1_elem = 'CameraParameters'
        else:
            return jsonify({'success': False, 'error': 'Invalid stereo parameter file format'}), 400

        cam1_data = stereo_params[cam1_elem]
        mtx1 = np.array(cam1_data['IntrinsicMatrix'])
        if stereo_params.get('Scheme', 'opencv') != 'opencv':
            mtx1 = mtx1.T
        dist1 = np.array([cam1_data['RadialDistortion'][:2] + cam1_data['TangentialDistortion'] + [cam1_data['RadialDistortion'][-1]]])

        # Get right camera parameters
        if 'CameraParameters2' in stereo_params:
            cam2_elem = 'CameraParameters2'
        else:
            return jsonify({'success': False, 'error': 'Missing right camera parameters'}), 400

        cam2_data = stereo_params[cam2_elem]
        mtx2 = np.array(cam2_data['IntrinsicMatrix'])
        if stereo_params.get('Scheme', 'opencv') != 'opencv':
            mtx2 = mtx2.T
        dist2 = np.array([cam2_data['RadialDistortion'][:2] + cam2_data['TangentialDistortion'] + [cam2_data['RadialDistortion'][-1]]])

        # Get rectification matrices
        R1 = np.eye(3)
        P1 = None
        Q = None
        if 'Rectification' in stereo_params:
            rect_data = stereo_params['Rectification']
            if 'RotationMatrix1' in rect_data:
                R1 = np.array(rect_data['RotationMatrix1'])
            if 'RotationMatrix2' in rect_data:
                R2 = np.array(rect_data['RotationMatrix2'])
            if 'ProjectionMatrix1' in rect_data:
                P1 = np.array(rect_data['ProjectionMatrix1'])
            if 'DisparityToDepth' in rect_data:
                Q = np.array(rect_data['DisparityToDepth'])
        else:
            logger.warning("No rectification matrices found, using identity")

        # Get baseline and focal length for depth computation
        baseline = None
        focal_length = mtx1[0, 0]

        # Try to compute baseline from translation
        if 'TranslationOfCamera2' in stereo_params:
            T = np.array(stereo_params['TranslationOfCamera2'])
            baseline = np.linalg.norm(T) * 1000  # Convert to mm

        # Load images
        left_img = cv2.imread(left_path, cv2.IMREAD_GRAYSCALE)
        right_img = cv2.imread(right_path, cv2.IMREAD_GRAYSCALE)

        if left_img is None or right_img is None:
            return jsonify({'success': False, 'error': 'Failed to load images'}), 400

        if left_img.shape != right_img.shape:
            return jsonify({'success': False, 'error': 'Images must have same dimensions'}), 400

        # Apply rectification if matrices are available
        if not np.allclose(R1, np.eye(3)) and P1 is not None and Q is not None:
            h, w = left_img.shape[:2]
            newcameramtx1, rect1 = cv2.getOptimalNewCameraMatrix(mtx1, dist1, (w, h), 1, (w, h))
            newcameramtx2, rect2 = cv2.getOptimalNewCameraMatrix(mtx2, dist2, (w, h), 1, (w, h))
            left_img = cv2.remap(left_img, rect1, newcameramtx1, cv2.INTER_LINEAR)
            right_img = cv2.remap(right_img, rect2, newcameramtx2, cv2.INTER_LINEAR)
        else:
            # Just undistort
            h, w = left_img.shape[:2]
            newcameramtx1, _ = cv2.getOptimalNewCameraMatrix(mtx1, dist1, (w, h), 1)
            newcameramtx2, _ = cv2.getOptimalNewCameraMatrix(mtx2, dist2, (w, h), 1)
            left_img = cv2.undistort(left_img, mtx1, dist1, None, newcameramtx1)
            right_img = cv2.undistort(right_img, mtx2, dist2, None, newcameramtx2)

        # Configure SGBM
        min_disparity = 0
        sbm = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=num_disparities,
            blockSize=block_size,
            P1=8 * 3 * block_size ** 2,
            P2=32 * 3 * block_size ** 2,
            disp12MaxDiff=disp12_max_diff,
            uniquenessRatio=uniqueness_ratio,
            speckleWindowSize=speckle_window_size,
            speckleRange=speckle_range,
            mode=cv2.StereoSGBM_MODE_SGBM_3WAY
        )

        logger.info(f"Computing disparity with SGBM: numDisparities={num_disparities}, blockSize={block_size}")

        # Compute disparity
        tic = time.time()
        disparity = sbm.compute(left_img, right_img)
        duration = time.time() - tic
        logger.info(f"Disparity computed in {duration:.2f} seconds")

        # Normalize disparity for display (0-255)
        disp_normalized = cv2.normalize(disparity / 16.0, None, 0, 255, cv2.NORM_MINMAX)
        disp_colored = cv2.applyColorMap(np.uint8(disp_normalized), cv2.COLORMAP_JET)

        # Convert disparity to float
        disp_float = disparity.astype(np.float32) / 16.0
        h, w = left_img.shape[:2]

        # Compute depth and 3D points
        depth = np.zeros((h, w), dtype=np.float32)
        points_3d = np.zeros((h, w, 3), dtype=np.float32)

        # Method 1: Using Q matrix (gives full 3D coordinates)
        if Q is not None:
            points_3d = cv2.reprojectImageTo3D(disp_float, Q)
            depth = points_3d[:, :, 2]  # Z coordinate

        # Method 2: Using baseline and focal length
        else:
            if baseline is not None:
                depth = (baseline * focal_length) / disp_float
            else:
                baseline = 120  # Default 120mm baseline
                depth = (baseline * focal_length) / disp_float

            # Reconstruct 3D points from depth
            fx = focal_length
            fy = mtx1[1, 1]
            cx = mtx1[0, 2]
            cy = mtx1[1, 2]

            y, x = np.meshgrid(np.arange(h), np.arange(w), indexing='ij')
            points_3d[:, :, 0] = (x - cx) * depth / fx
            points_3d[:, :, 1] = (y - cy) * depth / fy
            points_3d[:, :, 2] = depth

        # Clean up invalid values (including infinity and NaN)
        finite_mask = np.isfinite(depth) & (depth > 0)
        depth = np.where(finite_mask, depth, 0)
        depth = depth.astype(np.float32)

        # Store for point cloud export - only finite points
        points_3d = np.where(np.isfinite(points_3d), points_3d, 0)
        point_cloud_valid = points_3d[finite_mask]  # (N, 3) array of valid points

        # Normalize depth for display
        valid_depth = depth[depth > 0]
        if len(valid_depth) > 0:
            depth_min = np.percentile(valid_depth, 1)
            depth_max = np.percentile(valid_depth, 99)
            depth_clamped = np.clip(depth, depth_min, depth_max)
            depth_normalized = cv2.normalize(depth_clamped, None, 0, 255, cv2.NORM_MINMAX)
            depth_colored = cv2.applyColorMap(np.uint8(depth_normalized), cv2.COLORMAP_VIRIDIS)
        else:
            depth_colored = np.zeros_like(disp_colored)

        # Encode images as base64
        disp_enc = cv2.imencode('.png', disp_colored)[1].tobytes()
        depth_enc = cv2.imencode('.png', depth_colored)[1].tobytes()
        left_color = cv2.cvtColor(left_img, cv2.COLOR_GRAY2BGR)
        texture_enc = cv2.imencode('.png', left_color)[1].tobytes()

        # Compute statistics
        if len(valid_depth) > 0:
            # Filter out infinite and NaN values
            finite_depth = valid_depth[np.isfinite(valid_depth)]
            if len(finite_depth) > 0:
                min_depth = float(np.min(finite_depth))
                max_depth = float(np.max(finite_depth))
                mean_depth = float(np.mean(finite_depth))
            else:
                min_depth = 0.0
                max_depth = 0.0
                mean_depth = 0.0
        else:
            min_depth = 0.0
            max_depth = 0.0
            mean_depth = 0.0

        import base64
        from io import BytesIO
        import struct

        # Subsample point cloud for web (reduce by factor of N)
        subsample = 10
        if len(point_cloud_valid) > 50000:
            subsample = int(len(point_cloud_valid) / 50000) + 1

        points_subsampled = point_cloud_valid[::subsample]

        # Prepare point cloud data for Three.js
        points_data = {
            'positions': points_subsampled.tolist(),  # List of [x, y, z]
            'count': len(points_subsampled),
            'image_width': w,
            'image_height': h
        }

        return jsonify({
            'success': True,
            'disparity_image': f'data:image/png;base64,{base64.b64encode(disp_enc).decode()}',
            'depth_image': f'data:image/png;base64,{base64.b64encode(depth_enc).decode()}',
            'texture_image': f'data:image/png;base64,{base64.b64encode(texture_enc).decode()}',
            'disparity_download': base64.b64encode(disp_enc).decode(),
            'depth_download': base64.b64encode(depth_enc).decode(),
            'min_depth': min_depth,
            'max_depth': max_depth,
            'mean_depth': mean_depth,
            'num_valid_pixels': len(valid_depth),
            'point_cloud': points_data
        })

    except Exception as e:
        logger.exception(f"Disparity computation error: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

    finally:
        try:
            shutil.rmtree(temp_dir, ignore_errors=True)
        except:
            pass


if __name__ == '__main__':
    logger.info("=" * 60)
    logger.info("Camera Calibration Tool - Web Application")
    logger.info("=" * 60)
    logger.info("Server: http://0.0.0.0:5000")
    logger.info("Features:")
    logger.info("  - Mono Camera Calibration")
    logger.info("  - Stereo Camera Calibration")
    logger.info("  - Hand-Eye Calibration")
    logger.info("  - Stereo Disparity Computation")
    logger.info("=" * 60)
    app.run(host='0.0.0.0', port=5000, debug=True)
