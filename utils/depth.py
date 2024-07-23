import cv2
from loguru import logger
import numpy as np
import os
import threading
import time

def timer_decorator(func):
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        logger.debug(f'{func.__name__} took {end_time - start_time} seconds')
        return result
    return wrapper

def load_systemconfig(configPath: str):
    pass 

class SgbmCpu():
    def __init__(self, stereoParamPath:str, configPath:str, imageSize:tuple):
        self.P1 = None
        self.P2 = None
        self.blockSize = 3
        self.minDisparity = 0
        self.numDisparities = 16*16
        self.disp12MaxDiff = 1
        self.preFilterCap = 63
        self.uniquenessRatio = 15
        self.speckleWindowSize = 0
        self.speckleRange = 32
        self.mode = cv2.StereoSGBM_MODE_HH



if __name__ == "__main__":
    pass 