"""Flask Blueprint - Camera Calibration Tool"""

import os
import sys
import cv2
import numpy as np
from datetime import datetime
from flask import Blueprint, render_template, request, jsonify
from loguru import logger
from pathlib import Path
import tempfile
import shutil
from functools import wraps

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))
from utils.calib import CalibChessboard
from werkzeug.utils import secure_filename

calibration_bp = Blueprint('calibration', __name__, url_prefix='/calibration')


def allowed_file(filename):
    """Check file extension"""
    allowed = {'png', 'jpg', 'jpeg', 'bmp', 'webp'}
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in allowed


def log_request(view):
    """Wrap function to log requests and time taken"""
    @wraps(view)
    def inner(*args, **kwargs):
        start = datetime.now()
        result = view(*args, **kwargs)
        duration = (datetime.now() - start).total_seconds()
        logger.debug(f"{request.method} {request.path} - {duration:.2f}s")
        return result
    return inner


@calibration_bp.route('/')
@log_request
def index():
    return render_template('index.html')


@calibration_bp.route('/mono')
@log_request
def mono_calibration():
    return render_template('mono_calibration.html')


@calibration_bp.route('/mono/calibrate', methods=['POST'])
@log_request
def calibrate_mono():
    """Mono camera calibration endpoint"""
    try:
        rows = int(request.form.get('rows', 10))
        cols = int(request.form.get('cols', 7))
        cell_size = float(request.form.get('cellsize', 1.0))
        files = request.files.getlist('files')

        if not files or not all(allowed_file(f.filename) for f in files):
            return jsonify({'success': False, 'error': 'No valid images'})

        temp_dir = tempfile.mkdtemp()
        image_files = []

        try:
            timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
            for i, f in enumerate(files):
                if allowed_file(f.filename):
                    filename = f"{timestamp}{i:04d}_{secure_filename(f.filename)}"
                    filepath = os.path.join(temp_dir, filename)
                    f.save(filepath)
                    image_files.append(filepath)

            logger.info(f"Parameters: {rows}x{cols}, {cell_size}mm, {len(image_files)} images")

            calib = CalibChessboard(rows, cols, cell_size, use_mt=True)

            # Extract filenames from paths for calibration
            filenames = [os.path.basename(f) for f in image_files]

            result = calib.mono_calib(temp_dir, filenames)

            if result[0]:
                return jsonify({
                    'success': True,
                    'mtx': result[1].tolist(),
                    'dist': result[2].tolist(),
                    'error': result[3],
                    'rvecs': np.array(result[4]).tolist(),
                    'tvecs': np.array(result[5]).tolist(),
                    'reproj_error': result[6],
                    'packedt: skf result[8],
                    'filtered': jfn
                })
            else:
                return jsonify({'success': False, 'error': f"Calibration error: {result[11]}"})

        finally:
            shutil.rmtree(temp_dir, ignore_errors=True)

    except Exception as e:
        logger.error(f"Calibration error: {e}", exc_info=True)
        return jsonify({'success': False, 'error': str(e)})


@log_request
def stereo_calibration():
    """Stereo camera calibration endpoint"""
    return render_template('stereo_calibration.html')


@calibration_bp.route('/stereo/calibrate', methods=['POST'])
@log_request
def calibrate_stereo():
    """Stereo calibration endpoint"""
    try:
        rows = int(request.form.get('rows', 10))
        cols = int(request.form.get('cols', 7))
        cell_size = float(request.form.get('cellsize', 1.0))

        left_files = request.files.getlist('left_files')
        right_files = request.files.getlist('right_files')

        if not left_files or not right_files or len(left_files) != len(right_files):
            return jsonify({'success': False, 'error': 'Upload matching stereo pairs'})

        temp_left = tempfile.mkdtemp()
        temp_right = tempfile.mkdtemp()
        lfiles, rfiles = [], []

        try:
            timestamp = datetime.now().strftime("%Y%m%d%H%M%S")

            for i, f in enumerate(left_files):
                if allowed_file(f.filename):
                    filepath = f"{temp_left}/{timestamp}{i:04d}_{secure_filename(f.filename)}"
                    f.save(filepath)
                    lfiles.append(filepath)

            for i, f in enumerate(right_files):
                if allowed_file(f.filename):
                    filepath = f"{temp_right}/{timestamp}{i:04d}_{secure_filename(f.filename)}"
                    f.save(filepath)
                    rfiles.append(filepath)

            logger.info(f"Stereo: {rows}x{cols}, {cell_size}mm, {len(lfiles)} pairs")

            calib = CalibChessboard(rows, cols, cell_size, use_mt=True)

            lfnames = [os.path.basename(f) for f in lfiles]
            rfames = [os.path.basename(f) for f in rfiles]

            result = calib.stereo_calib(temp_left, temp_right, lfnames, rfames)

            if result[0]:
                return jsonify({
                    'success': True,
                    'mtx_l': result[1].tolist(),
                    'dist_l': result[2].tolist(),
                    'mtx_r': result[3].tolist(),
                    'dist_r': result[4].tolist(),
                    'R': result[5].tolist(),
                    'T': result[6].tolist(),
                    'E': result[7].tolist(),
                    'F': result[8].tolist(),
                    'rvecs': np.array(result[9]).tolist(),
                    'tvecs': np.array(result[10]).tolist(),
                    'error': result[11]
                })
            else:
                return jsonify({'success': False, 'error': f"Error: {result[18]}"})

        finally:
            shutil.rmtree(temp_left, ignore_errors=True)
            shutil.rmtree(temp_right, ignore_errors=True)

    except Exception as e:
        logger.error(f"Stereo error: {e}", exc_info=True)
        return jsonify({'success': False, 'error': str(e)})


@calibration_bp.route('/handeye')
@log_request
def handeye_calibration():
    """Hand-eye calibration page"""
    return render_template('handeye_calibration.html')


@calibration_bp.route('/disparity')
@log_request
def disparity():
    """Stereo disparity calculation page"""
    return render_template('disparity.html')