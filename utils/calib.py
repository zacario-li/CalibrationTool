import cv2
import numpy as np
import pandas as pd
import os
import time
import json
import math
from multiprocessing import Pool
from functools import partial
from loguru import logger
from scipy.spatial.transform import Rotation
from utils.checkerboard import detect_checkerboard
from utils.err import CalibErrType

def compute_rotation_angle(ax, xb):
    try:
        if ax.shape[0] != ax.shape[1] or xb.shape[0] != xb.shape[1]:
            raise ValueError("Both matrices must be square.")
        
        if np.linalg.det(ax) == 0:
            raise ValueError("Matrix ax must be invertible.")
        
        deltaR = xb @ np.linalg.inv(ax)
        tr_a = np.trace(deltaR)
        
        cos_theta = (tr_a -1)/2
        if cos_theta < -1 or cos_theta >1:
            raise ValueError("Trace calculation is out of bounds for acos.")

        theta = np.rad2deg(np.arccos(cos_theta))
        return theta
    except Exception as e:
        logger.warning(f"Error: {e}")
        return None


def combine_RT(R, Tx, Ty, Tz):
    M = np.hstack([R, [[Tx], [Ty], [Tz]]])
    M = np.vstack((M, [0, 0, 0, 1]))  # convert it to homogeneous matrix
    return M


def timer_decorator(func):
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        logger.debug(f'{func.__name__} took {end_time - start_time} seconds')
        return result
    return wrapper

def load_handeye_param(filename:str):
    with open(filename) as f:
        jstr = json.load(f)
    
    if 'AXXB' in jstr:
        ELEMENT_NAME = 'AXXB'
    else:
        ELEMENT_NAME = 'AXZB'
    
    he = jstr[f'{ELEMENT_NAME}']['Matrix']
    return np.array(he)

def load_camera_param(filename: str, need_trans=False, camera_id=False, need_rt=False, need_size=False):
    # load parameters
    with open(filename) as f:
        jstr = json.load(f)

    NEED_TRANS = need_trans
    if 'Scheme' in jstr:
        if jstr['Scheme'] != 'opencv':
            NEED_TRANS = True

    if 'CameraParameters' in jstr:
        ELEMENT_NAME = 'CameraParameters'
    elif camera_id is False and 'CameraParameters1' in jstr:
        ELEMENT_NAME = 'CameraParameters1'
    elif camera_id is True and 'CameraParameters2' in jstr:
        ELEMENT_NAME = 'CameraParameters2'
    else:
        logger.debug("json file load err")
        return None, None

    intri = jstr[f'{ELEMENT_NAME}']['IntrinsicMatrix']
    dist_r = jstr[f'{ELEMENT_NAME}']['RadialDistortion']
    dist_t = jstr[f'{ELEMENT_NAME}']['TangentialDistortion']

    mtx = np.array(intri)
    if NEED_TRANS:
        mtx = mtx.T
    dist = np.array(
        [dist_r[:2] + dist_t + [dist_r[-1]]]
    )
    if need_rt is not True:
        if need_size is not True:
            return mtx, dist
        else:
            w,h = None, None
            if 'ImageShape' in jstr:
                imageshape = jstr['ImageShape']
                w,h = imageshape[0], imageshape[1]
            return mtx, dist, w, h    
    else:
        if 'RotationOfCamera2' in jstr and 'TranslationOfCamera2' in jstr:
            rot2 = jstr['RotationOfCamera2']
            trans2 = jstr['TranslationOfCamera2']
        else:
            logger.debug("Can't find RotationOfCamera2 and TranslationOfCamera2 in json file")
            return mtx, dist, None, None
        return mtx, dist, np.array(rot2), np.array(trans2)


def quat_2_rot(q: np.array):
    q = q / np.linalg.norm(q)
    q0 = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]

    r00 = 1 - 2*(qy**2 + qz**2)
    r01 = 2*(qx*qy - q0*qz)
    r02 = 2*(qx*qz + q0*qy)
    r10 = 2*(qx*qy + q0*qz)
    r11 = 1 - 2*(qx**2 + qz**2)
    r12 = 2*(qy*qz - q0*qx)
    r20 = 2*(qx*qz - q0*qy)
    r21 = 2*(qy*qz + q0*qx)
    r22 = 1 - 2*(qx**2 + qy**2)

    R = np.array([[r00, r01, r02],
                  [r10, r11, r12],
                  [r20, r21, r22]])
    return R


def rot_2_quat(R: np.array):
    # Convert to quaternion
    tr = np.trace(R)
    q = np.zeros(4)

    if tr > 0:
        S = np.sqrt(tr+1.0) * 2
        q[0] = 0.25 * S
        q[1] = (R[2, 1] - R[1, 2]) / S
        q[2] = (R[0, 2] - R[2, 0]) / S
        q[3] = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q[0] = (R[2, 1] - R[1, 2]) / S
        q[1] = 0.25 * S
        q[2] = (R[0, 1] + R[1, 0]) / S
        q[3] = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q[0] = (R[0, 2] - R[2, 0]) / S
        q[1] = (R[0, 1] + R[1, 0]) / S
        q[2] = 0.25 * S
        q[3] = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q[0] = (R[1, 0] - R[0, 1]) / S
        q[1] = (R[0, 2] + R[2, 0]) / S
        q[2] = (R[1, 2] + R[2, 1]) / S
        q[3] = 0.25 * S
    return q


def euler_2_quat(roll, pitch, yaw):
    q = np.zeros(4)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    q[0] = w, q[1] = x, q[2] = y, q[3] = z

    return q


def combine_RT(R, Tx, Ty, Tz):
    M = np.hstack([R, [[Tx], [Ty], [Tz]]])
    M = np.vstack((M, [0, 0, 0, 1]))  # convert it to homogeneous matrix
    return M


class HandEye():
    def __init__(self):
        pass

    def generate_gripper2base_with_rvec_txt(self, filename: str, sensor_only=False, randomtest=False):
        data = np.loadtxt(filename)
        x = data[:, 0]
        y = data[:, 1]
        z = data[:, 2]
        rxyz = data[:, 3:]

        rot = Rotation.from_rotvec(rxyz)
        quat_array = rot.as_quat()
        quat_array[:, [0, 1, 2, 3]] = quat_array[:, [3, 0, 1, 2]]

        # reshape
        x = x.reshape(-1, 1)
        y = y.reshape(-1, 1)
        z = z.reshape(-1, 1)

        # Q = np.hstack((q0, qx, qy, qz))
        Q = quat_array
        Ts = np.hstack((x, y, z))*1000.0

        R = []
        T = []
        for rr, tt in zip(Q, Ts):
            r = quat_2_rot(rr)
            R.append(r)
            T.append(tt)
        return R, T

    def generate_gripper2base_with_euler_txt(self, filename: str, sensor_only=False, randomtest=False):
        data = np.loadtxt(filename)
        x = data[:, 0]
        y = data[:, 1]
        z = data[:, 2]
        roll = data[:, 3]
        pitch = data[:, 4]
        yaw = data[:, 5]

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - \
            np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + \
            np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - \
            np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        q0 = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + \
            np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        Q = np.hstack((q0, qx, qy, qz))
        Ts = np.hstack((x, y, z))

        R = []
        T = []
        for rr, tt in zip(Q, Ts):
            r = quat_2_rot(rr)
            R.append(r)
            T.append(tt)
        return R, T

    def generate_gripper2ndi_with_file(self, filename: str, sensor_only=False, randomtest=False):
        df = pd.read_csv(filename)
        q0 = np.array(df.iloc[:, 0])
        qx = np.array(df.iloc[:, 1])
        qy = np.array(df.iloc[:, 2])
        qz = np.array(df.iloc[:, 3])
        if sensor_only is False:
            tx = np.array(df.iloc[:, 4])
            ty = np.array(df.iloc[:, 5])
            tz = np.array(df.iloc[:, 6])
        else:
            if randomtest is False:
                tx = np.ones_like(q0)
                ty = np.ones_like(q0)
                tz = np.ones_like(q0)
            else:
                tx = np.random.rand(*q0.shape)
                ty = np.random.rand(*q0.shape)
                tz = np.random.rand(*q0.shape)

        q0 = q0.reshape(-1, 1)
        qx = qx.reshape(-1, 1)
        qy = qy.reshape(-1, 1)
        qz = qz.reshape(-1, 1)
        tx = tx.reshape(-1, 1)
        ty = ty.reshape(-1, 1)
        tz = tz.reshape(-1, 1)
        Q = np.hstack((q0, qx, qy, qz))
        if sensor_only is False:
            Ts = np.array(df.iloc[:, 4:7])
        else:
            Ts = np.hstack((tx, ty, tz))
        R = []
        T = []
        for rr, tt in zip(Q, Ts):
            r = quat_2_rot(rr)
            R.append(r)
            T.append(tt)
        return R, T

    def generate_gripper2ndi_with_raw_files(self, filepath: str, sensor_only=False, randomtest=False):
        pass

    def calib_axxb(self, r_g2n_list, t_g2n_list, r_b2c_list, t_b2c_list, calib_method):
        r_c2g, t_c2g = cv2.calibrateHandEye(
            r_g2n_list, t_g2n_list, r_b2c_list, t_b2c_list, method=calib_method
        )
        # val
        r_e, t_e = self.valaxxb(r_g2n_list, t_g2n_list,
                                r_b2c_list, t_b2c_list, r_c2g, t_c2g)
        return r_c2g, t_c2g, r_e, t_e

    def calib_axzb(self):
        pass

    def valaxxb(self, r_g2n_list, t_g2n_list, r_b2c_list, t_b2c_list, r_c2g, t_c2g):
        RT_c2g = combine_RT(r_c2g, float(
            t_c2g[0]), float(t_c2g[1]), float(t_c2g[2]))
        RT_b2c_1 = combine_RT(r_b2c_list[0], float(t_b2c_list[0][0]), float(
            t_b2c_list[0][1]), float(t_b2c_list[0][2]))
        RT_g2n_1 = combine_RT(r_g2n_list[0], float(t_g2n_list[0][0]), float(
            t_g2n_list[0][1]), float(t_g2n_list[0][2]))

        AX = []
        A = []
        XB = []
        B = []

        for i in range(1, len(r_b2c_list)):
            RT_b2c_2 = combine_RT(r_b2c_list[i], float(t_b2c_list[i][0]), float(
                t_b2c_list[i][1]), float(t_b2c_list[i][2]))
            RT_g2n_2 = combine_RT(r_g2n_list[i], float(t_g2n_list[i][0]), float(
                t_g2n_list[i][1]), float(t_g2n_list[i][2]))

            # let's check AX=XB
            # A = RT_g2b_2^{-1} @ RT_g2b_1
            # B = RT_b2c_2 @ RT_b2c_1^{-1}
            a = np.linalg.inv(RT_g2n_2) @ RT_g2n_1
            b = RT_b2c_2 @ np.linalg.inv(RT_b2c_1)
            ax = a @ RT_c2g
            xb = RT_c2g @ b

            A.append(a)
            B.append(b)
            AX.append(ax)
            XB.append(xb)

        rotation_err = self.re(AX, XB)
        translation_err = self.te(A, B, RT_c2g)
        return rotation_err, translation_err

    def te(self, a, b, x):
        '''
        err_te = 1/N\sum^N_{i=1}\| (R_A_i t_X) + t_A - (R_X t_B_i) - t_X \|
        '''
        tx = x[:3, 3].reshape(3, 1)
        rx = x[:3, :3]
        sum_te = []

        for ai, bi in zip(a, b):
            temp = ai[:3, :3]@tx + \
                ai[:3, 3].reshape(3, 1) - rx@(bi[:3, 3].reshape(3, 1)) - tx
            sum_te.append(temp)
        sum_np = np.asarray(sum_te).reshape(-1, 3)
        sum_temp_te = np.linalg.norm(sum_np, axis=1)
        return np.mean(sum_temp_te)

    def re(self, x1list: list, x2list: list):
        '''
        base on AX = XB
        δR = {R_X}{R_B}({R_A}{R_X})^{-1}, note that, here we only calcuate the Rotation part.
        '''
        Theta_err = []
        for x1, x2 in zip(x1list, x2list):
            theta = compute_rotation_angle(x1[:3, :3], x2[:3, :3])
            Theta_err.append(theta)
        return np.linalg.norm(np.asarray(Theta_err))/len(Theta_err)


class CalibChessboard():
    def __init__(self, row, col, cellsize, use_mt: bool = True, use_libcbdet = False):
        # use libcbdetect
        self.use_libcbdet=use_libcbdet
        # use multi-threading
        self.USE_MT = use_mt
        # checkerboard pattern
        self.ROW_COR = row-1
        self.COL_COR = col-1
        self.CELLSIZE = cellsize
        self.criteria = (cv2.TERM_CRITERIA_EPS +
                         cv2.TERM_CRITERIA_MAX_ITER, 3000, 0.00001)

        # 构建角点真实世界坐标系
        self.objp = np.zeros((self.COL_COR*self.ROW_COR, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.ROW_COR,
                                    0:self.COL_COR].T.reshape(-1, 2) * self.CELLSIZE

    # 单目校准
    @timer_decorator
    def mono_calib(self, rootpath: str, filelist: list):
        objpoints = []  # 3d points in real world space
        imgpoints = []  # 2d points in image plane.
        rejected_files = []  # 无法获取角点的图片列表
        calibrated_files = []  # 校准成功的文件列表

        for fname in filelist:
            img = cv2.imread(os.path.join(rootpath, fname))
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret, cors = self.find_corners(gray)
            if ret is not True:
                # remember the rejected files
                rejected_files.append(fname)
            else:
                calibrated_files.append(fname)
                objpoints.append(self.objp)
                imgpoints.append(cors)

        # 检查角点size是否为0
        if len(imgpoints) == 0:
            return False, None, None, None, None, None, None, None, None, None, CalibErrType.CAL_CORNER_DET_ERR

        ret, mtx, dist, rvecs, tvecs, stdintri, stdextri, perverrs = cv2.calibrateCameraExtended(
            objpoints, imgpoints, gray.shape[::-1], None, None)        

        # TODO evaluate the results
        return ret, mtx, dist, rvecs, tvecs, perverrs, rejected_files, calibrated_files, gray.shape[::-1], imgpoints, CalibErrType.CAL_OK

    # parallen mono calib
    @timer_decorator
    def mono_calib_parallel(self, rootpath: str, filelist: str):
        objpoints = []  # 3d points in real world space
        imgpoints = []  # 2d points in image plane.
        rejected_files = []  # 无法获取角点的图片列表
        calibrated_files = []  # 校准成功的文件列表

        # 尝试读取一张图片，用于后续读取图片尺寸
        image_for_shape = cv2.imread(os.path.join(rootpath, filelist[0]), 0)

        with Pool(min(len(filelist), os.cpu_count()-1)) as p:
            results = p.map(
                partial(self._process_image_corners, rootpath), filelist)

        for fname, objp, cors, status in results:
            if status == 'rejected':
                rejected_files.append(fname)
            else:
                calibrated_files.append(fname)
                objpoints.append(objp)
                imgpoints.append(cors)
        
        # 检查角点size是否为0
        if len(imgpoints) == 0:
            return False, None, None, None, None, None, None, None, None, None, CalibErrType.CAL_CORNER_DET_ERR

        ret, mtx, dist, rvecs, tvecs, stdintri, stdextri, perverrs = cv2.calibrateCameraExtended(
            objpoints, imgpoints, image_for_shape.shape[::-1], None, None)

        # TODO evaluate the results
        return ret, mtx, dist, rvecs, tvecs, perverrs, rejected_files, calibrated_files, image_for_shape.shape[::-1], imgpoints, CalibErrType.CAL_OK

    # 双目校准
    @timer_decorator
    def stereo_calib(self, leftrootpath: str, rightrootpath: str, leftfilelist: list, rightfilelist: str):
        objpoints = []  # 3d points in real world space
        imgpoints_left = []  # 2d points in left image plane.
        imgpoints_right = []  # 2d points in right image plane.
        rejected_files = []  # 无法获取角点的图片列表
        calibrated_files = []  # 校准成功的文件列表

        for lf, rf in zip(leftfilelist, rightfilelist):
            leftimg = cv2.imread(f'{os.path.join(leftrootpath,lf)}', 0)
            rightimg = cv2.imread(f'{os.path.join(rightrootpath,rf)}', 0)
            ret_l, cors_l = self.find_corners(leftimg)
            ret_r, cors_r = self.find_corners(rightimg)
            if ret_l is not True or ret_r is not True:
                rejected_files.append([lf, rf])
            else:
                calibrated_files.append([lf, rf])
                objpoints.append(self.objp)
                imgpoints_left.append(cors_l)
                imgpoints_right.append(cors_r)
        
        # 检查角点size是否为0
        if len(imgpoints_left) == 0 or len(imgpoints_right) == 0:
            return False, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, CalibErrType.CAL_CORNER_DET_ERR
        # single calibrate
        ret_l, mtx_l, dist_l, rvecs_l, tvecs_l, stdintri_l, stdextri_l, pererr = cv2.calibrateCameraExtended(
            objpoints, imgpoints_left, leftimg.shape[::-1], None, None)
        ret_r, mtx_r, dist_r, rvecs_r, tvecs_r, stdintri_r, stdextri_r, pererr = cv2.calibrateCameraExtended(
            objpoints, imgpoints_right, rightimg.shape[::-1], None, None)
        # stereo calibrate
        # ret, mtx_l0, dist_l0, mtx_r0, dist_r0, R, T, E, F = cv2.stereoCalibrate(
        #     objpoints, imgpoints_left, imgpoints_right, mtx_l, dist_l, mtx_r, dist_r, leftimg.shape[::-1], criteria=self.criteria)
        # 创建旋转矩阵和平移向量的初始值
        R = np.eye(3)  # 3x3的单位矩阵
        T = np.zeros((3, 1))  # 3x1的零向量
        ret, mtx_l0, dist_l0, mtx_r0, dist_r0, R, T, E, F, rvecs, tvecs, pererr = cv2.stereoCalibrateExtended(
            objpoints, imgpoints_left, imgpoints_right, mtx_l, dist_l, mtx_r, dist_r, leftimg.shape[::-1], R, T, criteria=self.criteria)

        return ret, mtx_l0, dist_l0, mtx_r0, dist_r0, R, T, E, F, rvecs, tvecs, pererr, rejected_files, calibrated_files, leftimg.shape[::-1], imgpoints_left, imgpoints_right, CalibErrType.CAL_OK

    @timer_decorator
    def stereo_calib_parallel(self, leftrootpath: str, rightrootpath: str, leftfilelist: list, rightfilelist: list):
        objpoints = []  # 3d points in real world space
        imgpoints_left = []  # 2d points in left image plane.
        imgpoints_right = []  # 2d points in right image plane.
        rejected_files = []  # 无法获取角点的图片列表
        calibrated_files = []  # 校准成功的文件列表

        # 尝试读取一张图片，用于后续读取图片尺寸
        image_for_shape = cv2.imread(
            os.path.join(leftrootpath, leftfilelist[0]), 0)

        with Pool(min(len(leftfilelist), os.cpu_count())) as p:
            results = p.map(self._stereo_process_image_corners, [(lf, rf, leftrootpath, rightrootpath) for lf, rf in zip(
                leftfilelist, rightfilelist)])

        for lfname, rfname, lcors, rcors, status in results:
            if status == 'rejected':
                rejected_files.append([lfname, rfname])
            else:
                calibrated_files.append([lfname, rfname])
                objpoints.append(self.objp)
                imgpoints_left.append(lcors)
                imgpoints_right.append(rcors)
        # 检查角点size是否为0
        if len(imgpoints_left) == 0 or len(imgpoints_right) == 0:
            return False, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, CalibErrType.CAL_CORNER_DET_ERR
        
        # single calibrate for each camera
        ret_l, mtx_l, dist_l, rvecs_l, tvecs_l, stdintri_l, stdextri_l, pererr = cv2.calibrateCameraExtended(
            objpoints, imgpoints_left, image_for_shape.shape[::-1], None, None)
        ret_r, mtx_r, dist_r, rvecs_r, tvecs_r, stdintri_r, stdextri_r, pererr = cv2.calibrateCameraExtended(
            objpoints, imgpoints_right, image_for_shape.shape[::-1], None, None)

        # 创建旋转矩阵和平移向量的初始值
        R = np.eye(3)  # 3x3的单位矩阵
        T = np.zeros((3, 1))  # 3x1的零向量
        ret, mtx_l0, dist_l0, mtx_r0, dist_r0, R, T, E, F, rvecs, tvecs, pererr = cv2.stereoCalibrateExtended(
            objpoints, imgpoints_left, imgpoints_right, mtx_l, dist_l, mtx_r, dist_r, image_for_shape.shape[::-1], R, T, criteria=self.criteria)

        return ret, mtx_l0, dist_l0, mtx_r0, dist_r0, R, T, E, F, rvecs, tvecs, pererr, rejected_files, calibrated_files, image_for_shape.shape[::-1], imgpoints_left, imgpoints_right, CalibErrType.CAL_OK

    # 重投影误差

    def rpje(self, corners, r, t, cameraMatrix, distCoeffs):
        points_number = self.ROW_COR*self.COL_COR
        imgpts, _ = cv2.projectPoints(
            self.objp, r, t, cameraMatrix, distCoeffs)

        err = np.linalg.norm(
            (imgpts.reshape(points_number, -1) - corners.reshape(points_number, -1)), axis=1)
        sum_err = np.sum(err**2)
        err_rms = np.sqrt(sum_err/points_number)
        return err_rms

    # 查找角点
    def find_corners(self, grayimg: np.array):
        if self.use_libcbdet: # https://www.cvlibs.net/software/libcbdetect/
            cors , score = detect_checkerboard(grayimg, (self.COL_COR, self.ROW_COR))
            if score < 1.0:
                return True, cors.astype(np.float32)
            else:
                return False, None
        else:
            ret, sub_corners = cv2.findChessboardCornersSB(
                grayimg, (self.ROW_COR, self.COL_COR), cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_MARKER)
            if ret is True:
                # remove cornerSubPix call due to findChessboardCornersSB already included the subpix
                # sub_corners = cv2.cornerSubPix(
                #     grayimg, sub_corners, (19, 19), (-1, -1), self.criteria)
                return ret, sub_corners
            else:
                return ret, None

    # 计算单张棋盘格的R,T
    def calculate_img_rt(self, grayimg, cameraMatrix, distCoeffs, vis=False):
        _, cors = self.find_corners(grayimg)
        if _ is not True:
            return None, None, None

        ret, rvecs, tvecs, inliers = cv2.solvePnPRansac(
            self.objp, cors.reshape(-1, 2), cameraMatrix, distCoeffs)
        if vis is True:
            imgpts, _ = cv2.projectPoints(
                self.objp, rvecs, tvecs, cameraMatrix, distCoeffs)
            ret = None
            img = cv2.drawChessboardCorners(
                grayimg, (self.ROW_COR, self.COL_COR), cors, ret)
            img = cv2.drawChessboardCorners(
                grayimg, (self.ROW_COR, self.COL_COR), imgpts, ret)
            cv2.imshow('image', img)
            cv2.waitKey(1)

        R, _ = cv2.Rodrigues(rvecs)

        return R, tvecs, cors

    def calculate_img_rt_mono(self, args):
        rej_flag = False
        root, filename, mtx, dist = args
        img = cv2.imread(os.path.join(root, filename), 0)
        R, tvecs, cors = self.calculate_img_rt(img, mtx, dist)
        # 记录无法检测角点的文件名
        if R is None:
            rej_flag = True
        return (R, tvecs, rej_flag, cors)

    # 多线程计算棋盘格R,T
    @timer_decorator
    def calculate_img_rt_parallel(self, root, imagelist, mtx, dist):
        with Pool(min(len(imagelist), os.cpu_count())) as p:
            results = p.map(self.calculate_img_rt_mono, [
                            (root, filename, mtx, dist) for filename in imagelist])
        return results

    # 画角点
    def draw_corners(self, img: np.array, corners, pattern_found=True):
        cv2.drawChessboardCorners(
            img, (self.ROW_COR, self.COL_COR), corners, pattern_found)

    # 画箭头
    def draw_arrows(self, img:np.array, pts1, pts2):
        # 创建一个箭头上色的列表，不同长度的箭头上不同的颜色
        start_color = np.array([255, 0, 0])
        end_color = np.array([0,0,255])
        gradient_colors = []
        for i in range(100):
            color = (start_color*(100-i)+end_color*i)/100
            gradient_colors.append(color.astype(int))
        
        for i in range(len(pts1)):
            pt1 = tuple(pts1[i])
            pt2 = tuple(pts2[i])
            # 因为重投影误差都是亚像素差别，无法在画面上呈现，故放大100倍
            dx = (pt2[0] - pt1[0])*100
            dy = (pt2[1] - pt1[1])*100
            pt1_new = (int(pt1[0]), int(pt1[1]))
            pt2_new = (int(pt1[0]+dx), int(pt1[1]+dy))
            # color index [0~99]
            color_idx = min(99, int(math.sqrt((dx*dx + dy*dy))))
            color = (int(gradient_colors[color_idx][0]),int(gradient_colors[color_idx][1]),int(gradient_colors[color_idx][2]))
            cv2.arrowedLine(img, pt1_new, pt2_new, color,2)
    
    # parallel processing the image
    def _process_image_corners(self, rootpath: str, fname: str):
        img = cv2.imread(os.path.join(rootpath, fname), 0)
        ret, cors = self.find_corners(img)
        if ret is not True:
            return (fname, None, None, 'rejected')
        else:
            return (fname, self.objp, cors, 'calibrated')

    # stereo parellel processing the image
    def _stereo_process_image_corners(self, args):
        lfname, rfname, lrootpath, rrootpath = args
        limg = cv2.imread(os.path.join(lrootpath, lfname), 0)
        rimg = cv2.imread(os.path.join(rrootpath, rfname), 0)
        lret, lcors = self.find_corners(limg)
        rret, rcors = self.find_corners(rimg)
        if lret is not True or rret is not True:
            return (lfname, rfname, None, None, 'rejected')
        else:
            return (lfname, rfname, lcors, rcors, 'calibrated')


if __name__ == "__main__":
    cb = CalibChessboard(9, 12, 5.0)
    mtx, dist = load_camera_param('stereoParam12x9.json')
    gray_img = cv2.imread(
        'C:/Users/lzj/Desktop/1013/eyeHand20231013/20231013094245L.png', 0)
    R, tvecs, cors = cb.calculate_img_rt(gray_img, mtx, dist)
    pass
