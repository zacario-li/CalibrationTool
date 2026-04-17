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

import sys, os, tempfile, shutil
from datetime import datetime
from flask import Flask, render_template, request, jsonify, make_response
from werkzeug.utils import secure_filename
from loguru import logger
from pathlib import Path
import cv2, numpy as np, json

# Calibration utilities
sys.path.insert(0, str(Path(__file__).parent.parent))
from utils.calib import CalibChessboard, HandEye, load_camera_param
from utils.ophelper import combine_RT

# Initialize Flask app
app = Flask(__name__, template_folder='templates', static_folder='static')

# Configuration
app.config['MAX_CONTENT_LENGTH'] = 500 * 1024 * 1024
app.config['SECRET_KEY'] = 'dev-secret-key'
app.config['ALLOWED_EXTENSIONS'] = {'png', 'jpg', 'jpeg', 'bmp', 'webp', 'csv', 'txt', 'json'}

# Setup logging
logger.remove()
logger.add(sys.stdout, format="{level} | {time:HH:mm:ss} | {message}", level="DEBUG")

# Helper functions
def allowed_file(filename):
    if not filename or '.' not in filename:
        return False
    return filename.rsplit('.', 1)[1].lower() in app.config['ALLOWED_EXTENSIONS']


@app.context_processor
def context_vars():
    return {'app_name': 'Calibration Tool', 'current_year': datetime.now().year}


# Frontend routes
@app.route('/')
def index():
    return render_template('index.html')


@app.route('/calibration/mono')
def mono_page():
    return render_template('mono_calibration.html')


@app.route('/calibration/stereo')
def stereo_page():
    return render_template('stereo_calibration.html')


@app.route('/calibration/handeye')
def handeye_page():
    return render_template('handeye_calibration.html')


@app.route('/calibration/disparity')
def disparity_page():
    return render_template('disparity.html')


# Mono calibration
@app.route('/api/mono/calibrate', methods=['POST'])
def mono_calibrate():
    try:
        rows = int(request.form.get('rows', 10))
        cols = int(request.form.get('cols', 7))
        cell_size = float(request.form.get('cellsize', 1.0))
        use_libcbdet = request.form.get('use_libcbdet', 'false') == 'true'
        files = request.files.getlist('files')

        if not files or not all(allowed_file(f.filename) for f in files):
            return jsonify({'success': False, 'error': 'No valid images'})

        temp_dir = tempfile.mkdtemp()
        image_files = []
        timestamp = datetime.now().strftime("%Y%m%d%H%M%S%f")

        for i, f in enumerate(files):
            if allowed_file(f.filename):
                path = os.path.join(temp_dir, f"{timestamp}{i:04d}_{f.filename}")
                f.save(path)
                image_files.append(path)

        calib = CalibChessboard(rows, cols, cell_size, use_mt=True, use_libcbdet=use_libcbdet)
        filenames = [os.path.basename(f) for f in image_files]
        result = calib.mono_calib(temp_dir, filenames)

        if result[0]:
            return jsonify({
                'success': True,
                'intrinsic': result[1].tolist(),
                'distortion': result[2].tolist(),
                'reproj_error': float(result[3]),
                'rvecs': np.array(result[4]).tolist(),
                'tvecs': np.array(result[5]).tolist(),
                'shape': result[8],
                'processed': len(filenames) - len(result[7]),
                'rejected': result[7]
            })
        return jsonify({'success': False, 'error': str(result[11])})

    except Exception as e:
        logger.error("Error: " + str(e), exc_info=True)
        return jsonify({'success': False, 'error': str(e)})

    finally:
        if 'temp_dir' in locals():
            shutil.rmtree(temp_dir, ignore_errors=True)


# Stereo calibration
@app.route('/api/stereo/calibrate', methods=['POST'])
def stereo_calibrate():
    try:
        rows = int(request.form.get('rows', 10))
        cols = int(request.form.get('cols', 7))
        cell_size = float(request.form.get('cellsize', 1.0)))
        left_files = request.files.getlist('left_files')
        right_files = request.files.getlist('right_files')

        if not left_files or not right_files or len(left_files) != len(right_files):
            return jsonify({'success': False, 'error': 'Mismatched pairs'})

        temp_left, temp_right = tempfile.mkdtemp(), tempfile.mkdtemp()
        lfiles, rfiles = [], []
        timestamp = datetime.now().strftime("%Y%m%d%H%M%S%f")

        for i, f in enumerate(left_files):
            if allowed_file(f.filename):
                path = os.path.join(temp_left, f"{timestamp}{i:04d}_{f.filename}")
                f.save(path)
                lfiles.append(path)

        for i, f in enumerate(right_files):
            if allowed_file(f.filename):
                path = os.path.join(temp_right, f"{timestamp}{i:04d}_{f.filename}")
                f.save(path)
                rfiles.append(path)

        calib = CalibChessboard(rows, cols, cell_size, use_mt=True)
        lfnames = [os.path.basename(f) for f in lfiles]
        rfnames = [os.path.basename(f) for f in rfiles]
        result = calib.stereo_calib(temp_left, temp_right, lfnames, rfnames)

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
        logger.error("Error: " + str(e), exc_info=True)
        return jsonify({'success': False, 'error': str(e)})

    finally:
        if 'temp_left' in locals(): shutil.rmtree(temp_left, ignore_errors=True)
        if 'temp_right' in locals(): shutil.rmtree(temp_right, ignore_errors=True)


# Hand-eye calibration
@app.route('/api/handeye/calibrate', methods=['POST'])
def handeye_calibrate():
    try:
        rows = int(request.form.get('rows', 10))
        cols = int(request.form.get('cols', 7))
        cell_size = float(request.form.get('cellsize', 1.0)))
        method = request.form.get('method', 'HORAUD')
        quat = request.form.get('quat_flag', 'true') == 'true'
        trans = request.form.get('trans_flag', 'false') == 'true'
        cam_id = request.form.get('cam_id', 'false') == 'true'
        use_libcbdet = request.form.get('use_libcbdet', 'false') == 'true'

        a_file = request.files['a_file']
        b_files = request.files.getlist('b_files')
        cam_param = request.files['cam_param']

        temp_dir = tempfile.mkdtemp()
        timestamp = datetime.now().strftime("%Y%m%d%H%M%S%f")

        # Save files
        ext = 'csv' if quat else 'txt'
        a_path = os.path.join(temp_dir, f"{timestamp}_A.{ext}")
        a_file.save(a_path)

        image_files = []
        for i, f in enumerate(b_files):
            if allowed_file(f.filename):
                path = os.path.join(temp_dir, f"{timestamp}{i:04d}_{f.filename}")
                f.save(path)
                image_files.append(path)

        cam_path = os.path.join(temp_dir, f"{timestamp}_cam.json")
        cam_param.save(cam_path)

        # Initialize
        he = HandEye()
        cb = CalibChessboard(rows, cols, cell_size, use_mt=True, use_libcbdet=use_libcbdet)
        cam_mtx, cam_dist = load_camera_param(cam_path, trans, cam_id)

        # Load A matrix
        if quat:
            r_g2n, t_g2n = he.generate_gripper2ndi_with_file(a_path, False)
        else:
            r_g2n, t_g2n = he.generate_gripper2base_with_rvec_txt(a_path, False)

        # Get B matrix
        filenames = [os.path.basename(f) for f in image_files]
        results = cb.calculate_img_rt_parallel(temp_dir, filenames, cam_mtx, cam_dist)

        r_b2c = [r[0] for r in results if r[0] is not None]
        t_b2c = [r[1] for r in results if r[1] is not None]

        if len(r_g2n) != len(r_b2c):
            return jsonify({'success': False, 'error': 'Data size mismatch'})

        # Calibration
        calib_methods = {
            'TSAI': cv2.CALIB_HAND_EYE_TSAI,
            'PARK': cv2.CALIB_HAND_EYE_PARK,
            'HORAUD': cv2.CALIB_HAND_EYE_HORAUD,
            'ANDREFF': cv2.CALIB_HAND_EYE_ANDREFF,
            'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS
        }
        method_id = calib_methods.get(method, cv2.CALIB_HAND_EYE_HORAUD)

        r_c2g, t_c2g, r_err, t_err = he.calib_axxb(r_g2n, t_g2n, r_b2c, t_b2c, method_id)
        X = combine_RT(r_c2g, t_c2g[0], t_c2g[1], t_c2g[2])

        return jsonify({
            'success': True,
            'X': X.tolist(),
            'r_error': float(r_err),
            't_error': float(t_err)
        })

    except Exception as e:
        logger.error("Error: " + str(e), exc_info=True)
        return jsonify({'success': False, 'error': str(e)})

    finally:
        if 'temp_dir' in locals():
            shutil.rmtree(temp_dir, ignore_errors=True)


# Disparity computation
@app.route('/api/disparity/compute', methods=['POST'])
def disparity_compute():
    try:
        left = request.files['left']
        right = request.files['right']

        min_disp = int(request.form.get('min_disp', 0))
        num_disp = int(request.form.get('num_disparities', 16)) * 16
        block_size = int(request.form.get('block_size', 3))

        left_bytes = np.frombuffer(left.read(), np.uint8)
        right_bytes = np.frombuffer(right.read(), np.uint8)
        left_img = cv2.imdecode(left_bytes, cv2.IMREAD_GRAY)
        right_img = cv2.imdecode(right_bytes, cv2.IMREAD_GRAY)

        sgbm = cv2.StereoSGBM(minDisparity=min_disp, numDisparities=num_disp, block_size=block_size)
        disparity = sgbm.compute(left_img, right_img) / 16

        disparity[disparity == min_disp] = 0

        disparity_color = cv2.applyColorMap(np.uint8(disparity * 256), cv2.COLORMAP_JET)
        _, disparity_encoded = cv2.imencode('.png', disparity_color)

        return jsonify({
            'success': True,
            'disparity': disparity_encoded.tobytes().hex(),
            'shape': list(left_img.shape),
            'min': float(disparity.min()),
            'max': float(disparity.max())
        })

    except Exception as e:
        logger.error("Error: " + str(e), exc_info=True)
        return jsonify({'success': False, 'error': str(e)})


# Download calibration result
@app.route('/api/download', methods=['GET'])
def download_calibration():
    try:
        raw = request.args.get('raw', 'mono')
        rows = int(request.args.get('rows', 10))
        cols = int(request.args.get('cols', 7))
        cell_size = float(request.args.get('cell_size', 1.0)))
        intrinsic = eval(request.args.get('intrinsic'))
        distortion = eval(request.args.get('distortion'))

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        data = {
            'timestamp': timestamp,
            'raw': raw,
            'rows': rows,
            'cols': cols,
            'cell_size': cell_size,
            'intrinsic': intrinsic,
            'distortion': distortion
        }

        if raw == 'stereo':
            data['R'] = eval(request.args.get('R'))
            data['T'] = eval(request.args.get('T'))
            data['E'] = eval(request.args.get('E'))
            data['F'] = eval(request.args.get('F'))
        elif raw == 'handeye':
            data['X'] = eval(request.args.get('X'))
            data['r_error'] = request.args.get('r_error')
            data['t_error'] = request.args.get('t_error')

        json_str = json.dumps(data, indent=4)
        filename = f"calib_{raw}_{timestamp}.json"

        return make_response(json_str, mimetype='application/json',
                           headers={'Content-Disposition': f'attachment; filename={filename}'})

    except Exception as e:
        logger.error("Error: " + str(e), exc_info=True)
        return jsonify({'error': str(e)})


# Main entry point
if __name__ == '__main__':
    logger.info("=" * 50)
    logger.info("Camera Calibration Tool - Web Application")
    logger.info("=" * 50)
    logger.info("Server: http://0.0.0.0:5000")
    logger.info("Features:")
    logger.info("  - Mono Camera Calibration")
    logger.info("  - Stereo Camera Calibration")
    logger.info("  - Hand-Eye Calibration")
    logger.info("  - Stereo Disparity Computation")
    logger.info("=" * 50)
    app.run(debug=True, host='0.0.0.0', port=5000, threaded=True)