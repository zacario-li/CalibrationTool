"""
Test script for calibration API
"""
import os
import requests
from requests import Session

def test_mono_calibration(image_dir, rows=11, cols=8, cellsize=5.0):
    """Test mono calibration with images from a directory."""
    url = "http://127.0.0.1:5000/api/mono/calibrate"

    # Get image files
    image_files = [os.path.join(image_dir, f) for f in os.listdir(image_dir)
                   if f.endswith('.png') or f.endswith('.jpg') or f.endswith('.jpeg')]

    print(f"Found {len(image_files)} images")

    files = []
    for i, img_path in enumerate(image_files[:20]):  # Use up to 20 images
        with open(img_path, 'rb') as f:
            files.append(('files', f.read(), os.path.basename(img_path)))

    print(f"Uploading {len(files)} images...")

    response = requests.post(url, files=files, data={
        'rows': rows,
        'cols': cols,
        'cellsize': cellsize,
        'use_libcbdet': 'false'
    })

    return response.json()

if __name__ == "__main__":
    image_dir = r"C:\Users\lizj\Desktop\workspace\12x9_checkerboard\l"
    result = test_mono_calibration(image_dir, rows=11, cols=8, cellsize=5.0)
    print(f"Success: {result.get('success', 'N/A')}")
    if 'error' in result:
        print(f"Error: {result['error']}")
    if 'processed' in result:
        print(f"Processed: {result['processed']}")
    if 'rejected' in result:
        print(f"Rejected: {result['rejected']}")
