# Camera Calibration Tool - Web Version

A web-based camera calibration tool for professional camera calibration tasks.

## Features

- **Mono Camera Calibration** - Standard monocular camera calibration
- **Stereo Camera Calibration** - Dual camera system calibration  
- **Hand-Eye Calibration** - Robot end-effector to camera transformation (UI complete, backend pending)
- **Stereo Disparity** - Depth measurement from stereo images (UI complete, backend pending)

## Quick Start

```bash
# Install dependencies
pip install -r requirements.txt

# Run application  
python app.py

# Access at http://localhost:5000
```

## Project Structure

```
webapp/
├── app.py                    // main application
├── config.py                 // configuration
├── routes.py                 // API routes
├── requirements.txt          // dependencies
├── static/                   // CSS, JS, images
├── templates/                // HTML templates
└── uploads/                  // temporary upload storage
```

## Template Files

- `base.html` - Base template with common layout
- `index.html` - Home page with feature cards
- `mono_calibration.html` - Mono camera calibration interface
- `stereo_calibration.html` - Stereo calibration interface
- `handeye_calibration.html` - Hand-eye calibration interface
- `disparity.html` - Stereo disparity interface

## API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/` | Home page |
| GET | `/calibration/mono` | Mono calibration page |
| POST | `/api/mono/calibrate` | Calibrate mono camera |
| GET | `/calibration/stereo` | Stereo calibration page |
| POST | `/api/stereo/calibrate` | Calibrate stereo cameras |

## Usage Example

```javascript
// Mono camera calibration
const form = document.getElementById('form');
const formData = new FormData(form);
formData.append('rows', 10);
formData.append('cols', 7);
formData.append('cellsize', 1.0);

// Add images
for (let i = 0; i < files.length; i++) {
    formData.append('files', files[i]);
}

fetch('/api/mono/calibrate', {
    method: 'POST',
    body: formData
})
.then(response => response.json())
.then(data => console.log(data));
```

## Architecture

- **Frontend**: Flask/Jinja2 templates with Bootstrap 5 styling
- **Backend**: Flask web server with REST API
- **CV Processing**: OpenCV-Python using original `utils/calib.py` algorithms
- **Multithreading**: Multiprocessing for parallel image processing

## Implementation Status

| Feature | Completion |
|---------|------------|
| Mono Calibration | ✅ 100% |
| Stereo Calibration | ✅ 100% |
| Hand-eye UI | ✅ 100% |
| Hand-eye Backend | ⚠️ 30% (needs HandEye class integration) |
| Disparity UI | ✅ 100% |
| Disparity Backend | ⚠️ 30% (needs disparity algorithms) |
