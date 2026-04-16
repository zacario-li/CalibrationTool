# Camera Calibration Tool - Web Version

A web-based camera calibration tool supporting:
- Mono Camera Calibration
- Stereo Camera Calibration
- HandEye Calibration
- Stereo Disparity Treatment

## Prerequisites

- Python 3.8+
- Flask >= 3.0
- OpenCV >= 4.5
- numpy >= 1.24
- loguru >= 0.7

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
  ├── routes.py             # API routes
  ├── requirements.txt      # Dependencies
  ├── static/               # Static files
  │   └── css/
  ├── templates/            # HTML templates
  │   ├── base.html
  │   ├── index.html
  │   ├── mono_calibration.html
  │   ├── stereo_calibration.html
  │   ├── handeye_calibration.html
  │   └── disparity.html
  └── uploads/              # Temporary storage
