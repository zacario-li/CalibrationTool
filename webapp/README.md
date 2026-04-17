# Camera Calibration Tool - Web Version

A comprehensive web-based camera calibration tool built with Flask. This project successfully converts the original wxPython desktop application to a full-stack web architecture.

## Features

| Feature | Description | Implementation Status |
|---------|-------------|---------------------|
| Mono Camera Calibration | Standard monocular camera calibration using checkerboard detection | ✅ Complete |
| Stereo Camera Calibration | Dual camera system calibration with full rectification pipeline | ✅ Complete |
| Hand-Eye Calibration | Robot end-effector to camera transformation (AXXB formulation) | ✅ Complete |
| Stereo Disparity | Depth measurement from stereo image pairs using SGBM | ✅ Complete |

## Technology Stack

### Backend
- **Web Framework**: Flask 3.0
- **Computer Vision**: OpenCV-Python 4.8
- **Numerical Processing**: NumPy 1.24
- **Logging**: Loguru 0.7
- **Template Engine**: Jinja2 (Flask built-in)

### Frontend
- **UI Framework**: Bootstrap 5.3
- **Template System**: Jinja2
- **Charts**: Tabulator (optional for advanced viz)

### Architecture
```
┌─────────────────────────────────────────┐
│                Frontend                 │
│    ┌─────────────────────────────────┐  │
│    │       Jinja2 Templates          │  │
│    │    • Bootstrap 5 Styling         │  │
│    │    • Interactive UI Components  │  │
│    └─────────────────────────────────┘  │
└───────────────────────┬─────────────────┘
            HTTP/REST  │
                        ▼
┌─────────────────────────────────────────┐
│                Backend                  │
│    ┌─────────────────────────────────┐  │
│    │      Flask Web Server           │  │
│    │    • Request Routing             │  │
│    │    • REST API Endpoints          │  │
│    │    • File Upload Management     │  │
│    │    • Temp File Handling         │  │
│    └─────────────────────────────────┘  │
│                    │                     │
│    ┌───────────────┴─────────────────┐   │
│    │       CV Processing            │   │
│    │   • Checkerboard Detection     │   │
│    │   • Camera Calibration         │   │
│    │   • Hand-Eye Calib (AXXB)      │   │
│    │   • Disparity Computation       │   │
│    │   • Multiprocessing Support    │   │
│    └─────────────────────────────────┘   │
└─────────────────────────────────────────┘
```

## Project Structure

```
webapp/
├── app.py                    # Main Flask application
├── config.py                 # Configuration module
├── routes.py                 # API routes and blueprints
├── requirements.txt          # Python dependencies
├── test_calib.py            # Calibration test utilities
├── README.md                 # This documentation
├── static/
│   └── css/
│       └── style.css        # Custom CSS styles
├── templates/
│   ├── base.html            # Base layout template
│   ├── index.html           # Home page
│   ├── mono_calibration.html # Mono calibration interface
│   ├── stereo_calibration.html # Stereo calibration interface
│   ├── handeye_calibration.html # Hand-eye calibration interface
│   └── disparity.html        # Stereo disparity interface
└── uploads/                  # Temporary upload storage
```

## Installation and Setup

### Prerequisites
- Python 3.8+
- pip package manager

### Installation
```bash
# Install dependencies
pip install -r requirements.txt
```

### Running the Application
```bash
python app.py
```

The web application will start at: **http://localhost:5000**

## API Reference

### Endpoints Overview

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/` | Home page |
| GET | `/calibration/mono` | Mono calibration page |
| POST | `/api/mono/calibrate` | Perform mono calibration |
| GET | `/calibration/stereo` | Stereo calibration page |
| POST | `/api/stereo/calibrate` | Perform stereo calibration |
| GET | `/calibration/handeye` | Hand-eye calibration page |
| POST | `/api/handeye/calibrate` | Calibrate hand-eye system |
| GET | `/calibration/disparity` | Disparity page |
| POST | `/api/disparity/compute` | Compute stereo disparity |
| GET | `/api/download` | Download calibration results |

### Mono Camera Calibration

**POST** `/api/mono/calibrate`

**Parameters:**
| Name | Type | Default | Description |
|------|------|---------|-------------|
| `rows` | int | 10 | Checkerboard rows |
| `cols` | int | 7 | Checkerboard columns |
| `cellsize` | float | 1.0 | Cell size in mm |
| `files` | multiple | required | Calibration images |
| `use_libcbdet` | bool | false | Enhanced corner detection |

### Stereo Camera Calibration

**POST** `/api/stereo/calibrate`

**Parameters:**
| Name | Type | Description |
|------|------|-------------|
| `rows` | int | Checkerboard rows |
| `cols` | int | Checkerboard columns |
| `cellsize` | float | Cell size in mm |
| `left_files` | multiple | Left camera images |
| `right_files` | multiple | Right camera images |

### Hand-Eye Calibration (AXXB)

**POST** `/api/handeye/calibrate`

**Parameters:**
| Name | Type | Description |
|------|------|-------------|
| `rows` | int | Checkerboard rows |
| `cols` | int | Checkerboard columns |
| `cellsize` | float | Cell size in mm |
| `method` | string | Calibration method (TSAI, PARK, etc.) |
| `a_file` | file | Robot motion data (CSV/TXT) |
| `b_files` | multiple | Checkerboard images |
| `cam_param` | file | Camera parameters JSON |
| `use_rect` | bool | Use rectified images |
| `use_left` | bool | Use left camera |
| `use_libcbdet` | bool | Enhanced detection |

### Stereo Disparity

**POST** `/api/disparity/compute`

**Parameters:**
| Name | Type | Default | Description |
|------|------|---------|-------------|
| `left` | file | - | Left rectified image |
| `right` | file | - | Right rectified image |
| `min_disp` | int | 0 | Minimum disparity |
| `num_disparities` | int | 16 | Number of disparities |
| `block_size` | int | 3 | SGBM block size |

## Usage Examples

### JavaScript/AJAX Client
```javascript
// Mono calibration
const form = document.getElementById('form');
const form_data = new FormData(form);
form_data.append('rows', 10);
form_data.append('cols', 7);
form_data.append('cellsize', 1.0);

for (let i = 0; i < files.length; i++) {
    form_data.append('files', files[i]);
}

fetch('/api/mono/calibrate', {
    method: 'POST',
    body: form_data
})
.then(response => response.json())
.then(data => {
    if (data.success) {
        console.log('Calibration successful!');
        console.log('Intrinsic matrix:', data.intrinsic);
        console.log('Reprojection error:', data.reproj_error);
    }
});
```

## Configuration

Edit `config.py` for custom settings:
```python
class Config:
    SECRET_KEY = 'change-in-production'
    MAX_CONTENT_LENGTH = 500 * 1024 * 1024  # 500 MB
    DEBUG = True
```

## Implementation Highlights

### Features Implemented
1. **Multi-file upload support** for batch processing
2. **Parallel processing** with Python's multiprocessing module
3. **Enhanced corner detection** with libcbdetect integration
4. **Temporary file management** with automatic cleanup
5. **Comprehensive error handling** and exception management
6. **JSON API responses** with detailed metadata
7. **Session management** for user state persistence
8. **Template inheritance** for consistent UI
9. **Parameter validation** and input sanitization

### Calibration Algorithms
- **Mono**: Zhang's method with epipolar constraints
- **Stereo**: Extended Z hang calibration + iterative rectification
- **Hand-Eye**: AX=XB formulation (Tsai, Park, Hora, vad, andreff/Daniilidis
- **Disparity**: SGBM with configurable parameters

## Files

| File | Description |
|------|-------------|
| `app.py` | Main Flask application (complete system implementation) |
| `config.py` | Configuration settings module |
| `routes.py` | API routes blueprint with full endpoint implementations |
| `requirements.txt` | Python dependency list |
| `static/css/style.css` | Custom CSS styles for UI enhancement |
| `templates/*.html` | Jinja2 templates for all interfaces |
| `test_calib.py` | Calibration test utilities |

## Development Status

### Completed
✅ Flask backend architecture
✅ API endpoint implementations
✅ Frontend template design
✅ Image processing pipeline
✅ Parallel computation support
✅ Error handling system
✅ User authentication (basic)
✅ Documentation and examples

### Future Enhancements
- 🔐 Production-ready authentication
- 📊 Real-time WebSocket communication
- 🔔 Auto notifications progress updates
- 📈 Advanced data visualization
- 📱 Progressive Web App (PWA) support

## Credits

Original wxPython desktop application converted to web architecture. All UV algorithms preserved with implementation and performance improvement.
