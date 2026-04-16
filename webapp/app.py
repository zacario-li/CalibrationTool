"""
Flask Web Application - Camera Calibration Tool

Provides:
- Mono camera calibration using checkerboard
- Stereo camera calibration
- Hand-eye calibration for robotics
- Stereo disparity/depth measurement
"""

import sys, os
from datetime import datetime
from flask import Flask, render_template, request, jsonify, make_response
from werkzeug.utils import secure_filename
from loguru import logger
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))
from utils.calib import CalibChessboard

app = Flask(__name__, template_folder='templates', static_folder='static')
app.config['MAX_CONTENT_LENGTH'] = 500 * 1024 * 1024
app.config['SECRET_KEY'] = 'dev-secret-key'
app.config['ALLOWED_EXTENSIONS'] = {'png', 'jpg', 'jpeg', 'bmp', 'webp'}

logger.remove()
logger.add(sys.stdout, format="{level} | {time:HH:mm:ss} | {message}", level="DEBUG")


def allowed_file(filename):
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in app.config['ALLOWED_EXTENSIONS']


@app.context_processor
def context_vars():
    return {'app_name': 'Calibration Tool', 'current_year': datetime.now().year}


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


@app.route('/api/mono/calibrate', methods=['POST'])
def mono_calibrate():
    import tempfile, shutil, cv2, numpy as np
    try:
        rows = int(request.form.get('rows', 10))
        cols = int(request.form.get('cols', 7))
        cell_size = float(request.form.get('cellsize', 1.0))
        files = request.files.getlist('files')
        use_libcbdet = request.form.get('use_libcbdet', 'false') == 'true'

        logger.info(f"Mono calib: {rows}x{cols}, {cell_size}mm, libcbdet={use_libcbdet}")

        if not files or not all(allowed_file(f.filename) for f in files):
            return jsonify({'success': False, 'error': 'No valid images'})

        temp_dir = tempfile.mkdtemp()
        img_files = []
        ts = datetime.now().strftime("%Y%m%d%H%M%S%f")

        try:
            for i, f in enumerate(files):
                if allowed_file(f.filename):
                    path = os.path.join(temp_dir, f"{ts}{i:04d}_{secure_filename(f.filename)}")
                    f.save(path)
                    img_files.append(path)

            calib = CalibChessboard(rows, cols, cell_size, use_mt=True, use_libcbdet=use_libcbdet)
            result = calib.mono_calib(temp_dir, [os.path.basename(f) for f in img_files])

            if result[0]:
                return jsonify({
                    'success': True,
                    'intrinsic': result[1].tolist(),
                    'distortion': result[2].tolist(),
                    'reproj_error': float(result[3]),
                    'rvecs': np.array(result[4]).tolist(),
                    'tvecs': np.array(result[5]).tolist(),
                    'shape': result[8],
                    'processed': len(img_files) - len(result[7]),
                    'rejected': result[7]
                })
            return jsonify({'success': False, 'error': str(result[11])})
        finally:
            shutil.rmtree(temp_dir, ignore_errors=True)

    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/stereo/calibrate', methods=['POST'])
def stereo_calibrate():
    import tempfile, shutil, cv2, numpy as np
    try:
        rows = int(request.form.get('rows', 10))
        cols = int(request.form.get('cols', 7))
        cell_size = float(request.form.get('cellsize', 1.0))
        left_files = request.files.getlist('left_files')
        right_files = request.files.getlist('right_files')

        if not left_files or not right_files:
            return jsonify({'success': False, 'error': 'No images'})
        if len(left_files) != len(right_files):
            return jsonify({'success': False, 'error': 'Mismatched pairs'})

        temp_left, temp_right = tempfile.mkdtemp(), tempfile.mkdtemp()
        lfiles, rfiles = [], []
        ts = datetime.now().strftime("%Y%m%d%H%M%S%f")

        try:
            for i, f in enumerate(left_files):
                if allowed_file(f.filename):
                    path = os.path.join(temp_left, f"{ts}{i:04d}_{f.filename}")
                    f.save(path)
                    lfiles.append(path)

            for i, f in enumerate(right_files):
                if allowed_file(f.filename):
                    path = os.path.join(temp_right, f"{ts}{i:04d}_{f.filename}")
                    f.save(path)
                    rfiles.append(path)

            calib = CalibChessboard(rows, cols, cell_size, use_mt=True)
            result = calib.stereo_calib(temp_left, temp_right, [os.path.basename(f) for f in lfiles],
                                       [os.path.basename(f) for f in rfiles])

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
        finally:
            shutil.rmtree(temp_left, ignore_errors=True)
            shutil.rmtree(temp_right, ignore_errors=True)

    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/download', methods=['GET'])
def download():
    import json
    try:
        data = {
            'rows': int(request.args.get('rows', 10)),
            'cols': int(request.args.get('cols', 7)),
            'cell_size': float(request.args.get('cell_size', 1.0))),
            'intrinsic': eval(request.args.get('intrinsic')),
            'distortion': eval(request.args.get('distortion'))
        }
        return make_response(json.dumps(data, indent=4), 'application/json',
                           {'Content-Disposition': 'attachment; filename=calib_result.json'})
    except Exception as e:
        return jsonify({'error': str(e)})


if __name__ == '__main__':
    logger.info("Starting Calibration Tool Web Application")
    app.run(debug=True, host='0.0.0.0', port=5000, threaded=True)