from loguru import logger
import numpy as np
import os
import threading
import time
import cv2
from utils.calib import load_camera_param


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
    def __init__(self, stereoParamPath: str, config, imageSize: tuple = (1920, 1080)):
        # camera parameters
        self.cam1_mtx = None
        self.cam1_dist = None
        self.cam2_mtx = None
        self.cam2_dist = None
        self.rofCam2 = None
        self.tofCam2 = None

        self.map1x, self.map1y = None, None
        self.map2x, self.map2y = None, None

        # sgbm parameters
        self.mode = config[0] #cv2.StereoSGBM_MODE_HH
        self.blockSize = config[1] #1
        self.sgbmP1 = config[2] #1
        self.sgbmP2 = config[3] #128
        self.minDisparity = config[4] #0
        self.numDisparities = config[5] #256
        self.disp12MaxDiff = config[6] #1
        self.preFilterCap = config[7] #15
        self.uniquenessRatio = config[8] #5
        self.speckleWindowSize = config[9] #50
        self.speckleRange = config[10] #8

        # init
        self._init_camera(stereoParamPath)
        self.stereo = self._create_instance()

    def _init_camera(self, stereoParamPath: str):
        w,h = 1920, 1080
        self.cam1_mtx, self.cam1_dist, w, h = load_camera_param(stereoParamPath, need_size=True)
        self.cam2_mtx, self.cam2_dist, self.rofCam2, self.tofCam2 = load_camera_param(
            stereoParamPath, camera_id=True, need_rt=True)
        
        self.R1, self.R2, self.P1, self.P2, self.Q, _, _ = cv2.stereoRectify(
            self.cam1_mtx, self.cam1_dist,
            self.cam2_mtx, self.cam2_dist,
            (w,h),
            self.rofCam2, self.tofCam2
        )

        self.map1x, self.map1y = cv2.initUndistortRectifyMap(
            self.cam1_mtx, self.cam1_dist, self.R1, self.P1, (w,h), cv2.CV_32FC1)
        self.map2x, self.map2y = cv2.initUndistortRectifyMap(
            self.cam2_mtx, self.cam2_dist, self.R2, self.P2, (w,h), cv2.CV_32FC1)

    def _create_instance(self):
        stereo = cv2.StereoSGBM_create(
            minDisparity=self.minDisparity,
            numDisparities=self.numDisparities,
            blockSize=self.blockSize,
            P1=self.sgbmP1,
            P2=self.sgbmP2,
            disp12MaxDiff=self.disp12MaxDiff,
            preFilterCap=self.preFilterCap,
            uniquenessRatio=self.uniquenessRatio,
            speckleWindowSize=self.speckleWindowSize,
            speckleRange=self.speckleRange,
            mode=self.mode
        )
        return stereo
