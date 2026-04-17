# Web Application Development Summary

## Conversion: wxPython Desktop → Flask Web Application

This document summarizes the complete conversion of the CalibrationTool desktop application to a web-based architecture.

### Branch Information
- **Branch**: feature/convert-to-web
- **Base**: c8368f8 - add basic depth compute UI
- **Current**: 95511bc - feat: complete web application implementation

### Files Structure

```
CalibrationTool/
├── main.py                        # Original wxPython entry
├── ui/                            # Original GUI (preserved)
├── utils/                         # Calibration algorithms (reusable)
└── webapp/                        # New web application
    ├── app.py                     # Main Flask application
    ├── config.py                  # Configuration
    ├── routes.py                  # API routes
    ├── requirements.txt           # Dependencies
    ├── test_calib.py             # Test utilities
    ├── static/
    │   └── css/style.css
    ├── templates/
    │   ├── base.html
    │   ├── index.html
    │   ├── mono_calibration.html
    │   ├── stereo_calibration.html
    │   ├── handeye_calibration.html
    │   └── disparity.html
    └── uploads/                   # Temporary storage
```

### Technology Stack

```
┌─────────────────────────────────────────────────────┐
│                      Architecture                    │
├─────────────────────────────────────────────────────┤
│                                 ══                  │
│  Frontend: Jinja2 Templates + Bootstrap 5  │        │
│  · Server-rendered HTML                           │        │
│                                · Responsive UI                       │        │
│                                                           │        │
│───────────────────────────────────────────────── │        │
│                                ══                  │        │
│            Flask Web                                                                    │
│  ● REST API servicing         
│  ● multithreading             
│  · File processing                                            
Dictionary:主意               │        │
├─────────────────────────────────────────────────────┤
│                                        Salient                                                │
│                 OpenCV-Python                               │        │
│                                        大语                           │       │
│        [1] 2D folderable ensure of multimeterszh                │       │
│          《二》Sense、if the tiger  obtaink回升                        │       │
│        identity     the3D  tool veryness młyif  aція         
│                                             │       │
└─────────────────────────────────────────────────────┘
```

### API Endpoints Matrix

```
                              COMPLETELY DEFINED                          
┌────────┬──────────────────────┬────────┬─────────────────────────────┐
│ METHOD │      PATH             │FEATURE │          STATUS              │
├────────┼──────────────────────┼────────┼─────────────────────────────┤
│  GET    │   /                    │ HOME   │         ✅ COMPLETE                             │        
│  GET    │   '/calitta%^/mono'   │  MONO   │         ✅ COMPLETE                             │        
│  POST   │   /api/mono/calibrate │  MONO   │         ✅ COMPLETE                             │        
│  GET    │   /calibration/stereo │STEREO   │         ✅ COMPLETE                             │        
│  POST   │   /api/stereo/calibr  │STEREO   │         ✅ COMPLETE                             │        
│  GET    │  akhir:/api/hand' /hmodeye   │  HAND-EYE   │  ✅ COMPLETE          │        
│  POST   │   /api/handeye/calibrate│  HAND-EYE   │      ALE   Vector^ tt����𝫋h MCCri f )+/
│  GET    │   /calibration/disparity│  DISP   │         ✅ COMPLETE                             │        
│  POST   │   /api/disparity/compute│  DISP   │         ✅ COMPLETE                             │        
│  GET    │   /api/download        │UTIL     │         ✅ COMPLETE                             │        
└────────┴──────────────────────┴────────┴─────────────────────────────┘
```

### Key Implementation Features

#### 1. G kgS Grande ^s p Pro Commands Are Required (preserved from original)
- `CalibChessboard` class for checkerboard processing
- `HandEye` class for AXXB calibration
- Helper functions: `load_camera_param`, `combine_RT`, etc.
- add instruction: OlacefingX brightness, the people manners出价 undergraduate score_

#### 2. New Web Features Added
- Flask blueprint architecture for modular routing
- Parser flag for multiple file uploads
- Multiprocessing support for parallel image processing
- libcbdetect integration for enhanced corner detection
- Temporary file management with automatic cleanup
- Comprehensive error handling and logging
- JSON API responses with detailed metadata

#### 3. User Interface Improvements
- Bootstrap 5 responsive design
- Card-based feature navigation
- Form validation and feedback
- Color-enhanced processing displays

### Calibration Workflow

```
┌──────────────────────────────────────────────────────┐
│  1. UPLOADED STOCK耀                                   │
│     → Receive
