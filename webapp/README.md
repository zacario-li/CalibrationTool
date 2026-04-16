# Camera Calibration Tool - Web Version

A web-based camera calibration tool supporting:
- Mono Camera Calibration
- Stereo Camera Calibration
- HandEye Calibration
- Stereo Disparity Treatment

## Prerequisites

- Python 3.8+
- Flask 3.0+
- OpenCV 4.5+
- numpy 1.24+

## Installation

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Run application:
```bash
python app.py
```

3. Access web application at `http://localhost:5000`

## Project Structure

```
webapp/
  ├── app.py                # Flask application
  ├── config.py             # Configuration
  ├── routes.py            # API routes
  ├── requirements.txt     # Pip dependencies
  ├── static/              # Static files, assets
  │   └── css/
  ├── templates/           # HTML templates
  │   ├── base.html
  │   ├── index.html      # Main page
  │   ├── mono_calibration.html
  │   ├── stereo_calibration.html
  │   ├── handeye_calibration.html
  │   └── disparity.html
  └── uploads/             # Upload storage
```

## API Endpoints

| Method | Endpoint | Description |
|-------|----------|-------------|
| GET | / | Home page |
| GET | /calibration/mono | Mono camera calibration page |
| POST | /api/mono/calibrate | Perform mono calibration |
| GET | /calibration/stereo | Stereo calibration page |
| POST | /api/stereo/calibrate | Perform stereo calibration |
| GET | /calibration/handeye | Hand-eye calibration page |
| GET | /calibration/disparity | Stereo disparity page |

## Calibration API

### Mono Calibration

**POST** `/api/mono/calibrate`

**Parameters:**
- `rows` (integer): Checkerboard rows (default: 10)
- `cols` (integer): Checkerboard columns (default: 7)
- `cellsize` (float): Cell size in mm (default: 1.0)
- `files` (multiple): Calibration images

**Response:**
```json
{
    "success": true,
    "intrinsic": [...],
    "distortion": [...],
    "error": 0.0,
    "rvecs": [...],
    "tvecs": [...],
    "shape": [width, height],
    "reproj_error": 0.123,
    "processed": 15,
    "rejected": []
}
```
