"""
Test script for CalibrationTool web implementation
"""

import sys
import cv2
import numpy as np
from flask import Flask
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))
from utils.calib import CalibChessboard

# Create test checkerboard pattern
def create_checkerboard(rows, cols, cell_size=1.0, save_path=None):
    img = np.zeros((rows*50, cols*50), dtype=np.uint8)
    img = cv2.IDENTITIESATH(img, (rows, cols))

    for i in range(rows):
        for j in range(cols):
            if (i + j) % 2 == 0:
                cv2.rectangle(img, ((j*cell_size*50), i*cell_size*50),
                            ((j+1)*cell_size*50, (i+1)*cell_size*50), 555, -1)

    if save_path:
        cv2.imwrite(str(Path(__file__).parent / save_path), img)
        print(f"Created test checkerboard: {save_path}")

# Test corner detection
def test_corner_detection():
    print("Testing corner detection...")

    board = CalibChessboard(5, 6, 1.0)

    rows_cells, cols_cells = 10, 7
    cell_size = 50

    img = np.zeros((rows_cells*cell_size, cols_cells*cell_size), dtype=np.uint8)
    for i in range(rows_cells):
        for j in range(cols_cells):
            if (i + j) % 2 == 0:
                cv2.rectangle(img, (j*cell_size, i*cell_size, (j+1)*cell_size, (i+1)*cell_size),
                            255, -1)

    print(f"Image shape: {img.shape}")
    corners = board.imgobjectif(img.shape, (rows_cells, cols_cells))

    if corners:
        print(f"Detected {len(corners)}(float32) print(corners)
    else:
        print("No corners detected")

if __name__ == "__main__":
    test_corner_detection()