"""Flask Blueprint - Camera Calibration Tool"""

import os
import sys
from datetime import datetime
from flask import Blueprint, render_template, request, jsonify
import cv2
import numpy as np
from loguru import logger
from pathlib import Path
import tempfile
import shutil
from functools import wraps

sys.path.insert(0, str(Path(__file__).parent.parent))
from utils.calib import CalibChessboard, HandEye, load_camera_param
from utils.ophelper import combine_RT
from werkzeug.utils import secure_filename

calibration_bp = Blueprint('calibration', __name__, url_prefix='/calibration')
ALLOWED = {'png', 'jpg', 'jpeg', 'bmp', 'webp', 'csv', 'txt', 'json'}


def allowed_file(f):
    return f and '.' in f and f.rsplit('.', 1)[1].lower() in ALLOWED


def log_request(view):
    @wraps(view)
    def inner(*args, **kwargs):
        result = view(*args, **kwargs)
        return result
    return inner


@calibration_bp.route('/')
def index():
    return render_template('index.html')


@calibration_bp.route('/mono')
def mono_calibration():
    return render_template('mono_calibration.html')


@calibration_bp.route('/stereo')
def stereo_calibration():
    return render_template('stereo_calibration.html')


@calibration_bp.route('/handeye')
def handeye_calibration():
    return render_template('handeye_calibration.html')


@calibration_bp.route('/disparity')
def disparity():
    return render_template('disparity.html')


@calibration_bp.route('/mono/calibrate', methods=['POST'])
def calibrate_mono():
    try:
        rows = int(request.form.get('rows', 10))
        cols = int(request.form.get('cols', 7))
        cell_sz = float(request.form.get('cellsize', 1.0))
        use_lib = request.form.get('use_libcbdet', 'false') == 'true'
        files = request.files.getlist('files')

        if not files or not all(allowed_file(f.filename) for f in files):
            return jsonify({'success': False, 'error': 'No valid images'})

        temp = tempfile.mkdtemp()
        path = []
        ts = datetime.now().strftime("%Y%m%d%H%M%S%f")

        for f in files:
            if allowed_file(f.filename):
                p = f"{temp}/{ts}{len(path):04d}_{f.filename}"
                f.save(p)
                path.append(p)

        calib = CalibChessboard(rows, cols, cell_sz, use_mt=True, use_libcbdet=use_lib)
        filenames = [os.path.basename(f) for f in path]
        result = calib.mono_calib(temp, filenames)

        if result[0]:
            return jsonify({
                'success': True,
                'intrinsic': result[1].tolist(),
                'distortion': result[2].tolist(),
                'error': float(result[3]),
                'rvecs': np.array(result[4]).tolist(),
                'tvecs': np.array(result[5]).tolist(),
                'shape': result[8],
                'processed': len(path),
                'rejected': result[7]
            })
        return jsonify({'success': False, 'error': str(result[11])})

    except Exception as e:
        logger.error(str(e), exc_info=True)
        return jsonify({'success': False, 'error': str(e)})
    finally:
        shutil.rmtree(temp if 'temp' in locals() else "", ignore_errors=True)


@calibration_bp.route('/stereo/calibrate', methods=['POST'])
def calibrate_stereo():
    try:
        rows = int(request.form.get('rows', 10))
        cols = int(request.form.get('cols', 7))
        cell_sz = float(request.form.get('cellsize', 1.0)))
        lf = request.files.getlist('left_files')
        rf = request.files.getlist('right_files')

        if not lf or not rf or len(lf) != len(rf):
            return jsonify({'success': False, 'error': 'Mismatched files'})

        temp_l, temp_r = tempfile.mkdtemp(), tempfile.mkdtemp()
        left, right = [], []
        ts = datetime.now().strftime("%Y%m%d%H%M%S%f")

        for i, f in enumerate(lf):
            if allowed_file(f.filename):
                path = f"{temp_l}/{ts}{i:04d}_{f.filename}"
                f.save(path)
                left.append(path)
        for i, f in enumerate(rf):
            if allowed_file(f.filename):
                path = f"{temp_r}/{ts}{i:04d}_{f.filename}"
                f.save(path)
                right.append(path)

        calib = CalibChessboard(rows, cols, cell_sz, use_mt=True)
        lfnames = [os.path.basename(f) for f in left]
        rfnames = [os.path.basename(f) for f in right]
        result = calib.stereo_calib(temp_l, temp_r, lfnames, rfnames)

        if result[0]:
            return jsonify({
                'success': True,
                'left': {'intrinsic': result[1].tolist(), 'distortion': result[2].tolist()},
                'right': {'intrinsic': result[3].tolist(), 'distortion': result[4].tolist()},
                'R': result[5].tolist(), 'T': result[6].tolist(),
                'E': result[7].tolist(), 'F': result[8].tolist(),
                'rvecs': np.array(result[9]).tolist(), 'tvecs': np.array(result[10]).tolist(),
                'error': float(result[11])
            })
        return jsonify({'success': False, 'error': str(result[18])})

    except Exception as e:
        logger.error(str(e), exc_info=True)
        return jsonify({'success': False, 'error': str(e)})
    finally:
        if 'temp_l' in locals(): shutil.rmtree(temp_l, ignore_errors=True)
        if 'temp_r' in locals(): shutil.rmtree(temp_r, ignore_errors=True)


@calibration_bp.route('/handeye/calibrate', methods=['POST'])
def calibrate_handeye():
    try:
        rows = int(request.form.get('rows', 10))
        cols = int(request.form.get('cols', 7))
        cell_sz = float(request.form.get('cellsize', 1.0)))
        method = request.form.get('method', 'HORAUD')
        quat = request.form.get('quat_flag', 'true') == 'true'
        trans = request.form.get('trans_flag', 'false') == 'true'
        cam_id = request.form.get('cam_id', 'false') == 'true'
        libcb = request.form.get('use_libcbdet', 'false') == 'true'

        a = request.files['a_file']
        b = request.files.getlist('b_files')
        cam = request.files['cam_param']

        temp = tempfile.mkdtemp()
        ts = datetime.now().strftime("%Y%m%d%H%M%S%f")

        a_path = f"{temp}/{ts}_A.{'csv' if quat else 'txt'}"
        a.save(a_path)

        imgs = []
        for i, f in enumerate(b):
            if allowed_file(f.filename):
                path = f"{temp}/{ts}{i:04d}_{f.filename}"
                f.save(path)
                imgs.append(path)

        cam_path = f"{temp}/{ts}_cam.json"
        cam.save(cam_path)

        he = HandEye()
        cb = CalibChessboard(rows, cols, cell_sz, use_mt=True, use_libcbdet=libcb)
        mtx, dist = load_camera_param(cam_path, trans, cam_id)

        if quat:
            rag2n, tg2n = he.generate_gripper2ndi_with_file(a_path, False)
        else:
            rag2n, tg2n = he.generate_gripper2base_with_rvec_txt(a_path, False)

        results = cb.calculate_img_rt_parallel(temp, [os.path.basename(f) for f in imgs], mtx, dist)
        rb2c = [r[0] for r in results if r[0] is not None]
        tb2c = [r[1] for r in results if r[1] is not None]

        if len(rag2n) != len(rb2c):
            return jsonify({'success': False, 'error': 'Size mismatch'})

        rc2g, tc2g, re, te = he.calib_axxb(rag2n, tg2n, rb2c, tb2c, getattr(cv2, f'CALIB_HAND_EYE_{method}', cv2.CALIB_HAND_EYE_HORAUD))
        X = combine_RT(rc2g, tc2g[0], tc2g[1], tc2g[2])

        return jsonify({'success': True, 'X': X.tolist(), 'r_err': re, 't_err': te})

    except Exception as e:
        logger.error(str(e), exc_info=True)
        return jsonify({'success': False, 'error': str(e)})
    finally:
        if 'temp' in locals(): shutil.rmtree(temp, ignore_errors=True)


@calibration_bp.route('/disparity/compute', methods=['POST'])
def compute_disparity():
    try:
        left = request.files['left']
        right = request.files['right']

        min_disp = int(request.form.get('min_disp', 0))
        num_disp = int(request.form.get('num_disparities', 16)) * 16
        block = int(request.form.get('block_size', 3))

        lb = np.frombuffer(left.read(), np.uint8)
        rb = np.frombuffer(right.read(), np.uint8)
        left = cv2.imdecode(lb, cv2.IMREAD_GRAY)
        right = cv2.imdecode(rb, cv2.IMREAD_GRAY)

        sgbm = cv2.StereoSGBM(minDisparity=min_disp, numDisparities=num_disp, blockSize=block)
        disp = sgbm.compute(left, right) / 16
        disp[disp == min_disp] = 0

        disp_col = cv2.applyColorMap(np.uint8(disp * 256), cv2.COLORMAP_JET)
        _, disp_enc = cv2.imencode('.png', disp_col)

        return jsonify({
            'success': True,
            'disp': disp_enc.tobytes().hex(),
            'shape': list(left.shape),
            'min': float(disp.min()),
            'max': float(disp.max())
        })

    except Exception as e:
        logger.error(str(e), exc_info=True)
        return jsonify({'success': False, 'error': str(e)})