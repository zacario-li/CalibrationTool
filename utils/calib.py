import cv2
import numpy as np
import os


def quat2rot(q: np.array):
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


def rot2quat(R: np.array):
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


def combineRT(R, Tx, Ty, Tz):
    M = np.hstack([R, [[Tx], [Ty], [Tz]]])
    M = np.vstack((M, [0, 0, 0, 1]))  # convert it to homogeneous matrix
    return M


class CalibChessboard():
    def __init__(self, row, col, cellsize):
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
    def single_calib(self, rootpath: str, filelist: list):
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

        ret, mtx, dist, rvecs, tvecs, stdintri, stdextri, perverrs = cv2.calibrateCameraExtended(
            objpoints, imgpoints, gray.shape[::-1], None, None)

        # TODO evaluate the results
        return ret, mtx, dist, rvecs, tvecs, perverrs, rejected_files, calibrated_files

    # 双目校准
    def stereo_calib(self, leftrootpath: str, rightrootpath: str, filelist: list):
        objpoints = []  # 3d points in real world space
        imgpoints_left = []  # 2d points in left image plane.
        imgpoints_right = []  # 2d points in right image plane.
        rejected_files = []  # 无法获取角点的图片列表
        calibrated_files = []  # 校准成功的文件列表

        for f in filelist:
            leftimg = cv2.imread(f'{os.path.join(leftrootpath,f)}', 0)
            rightimg = cv2.imread(f'{os.path.join(rightrootpath,f)}', 0)
            ret_l, cors_l = self.find_corners(leftimg)
            ret_r, cors_r = self.find_corners(rightimg)
            if ret_l or ret_r is not True:
                rejected_files.append(f)
            else:
                calibrated_files.append(f)
                objpoints.append(self.objp)
                imgpoints_left.append(cors_l)
                imgpoints_right.append(cors_r)
        # single calibrate
        ret_l, mtx_l, dist_l, rvecs_l, tvecs_l, stdintri_l, stdextri_l, perverrs_l = cv2.calibrateCameraExtended(
            objpoints, imgpoints_left, leftimg.shape[::-1], None, None)
        ret_r, mtx_r, dist_r, rvecs_r, tvecs_r, stdintri_r, stdextri_r, perverrs_r = cv2.calibrateCameraExtended(
            objpoints, imgpoints_right, rightimg.shape[::-1], None, None)
        # stereo calibrate
        ret, mtx_l0, dist_l0, mtx_r0, dist_r0, R, T, E, F, rvecs, tvecs, perviewerr = cv2.stereoCalibrateExtended(
            objpoints, imgpoints_left, imgpoints_right, mtx_l, dist_l, mtx_r, dist_r, leftimg.shape[::-1], criteria=self.criteria)

        return ret, mtx_l0, dist_l0, mtx_r0, dist_r0, R, T, E, F, rvecs, tvecs, perviewerr

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
        ret, sub_corners = cv2.findChessboardCornersSB(
            grayimg, (self.ROW_COR, self.COL_COR), cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_MARKER)

        if ret is True:
            sub_corners = cv2.cornerSubPix(
                grayimg, sub_corners, (19, 19), (-1, -1), self.criteria)
            return ret, sub_corners
        else:
            return ret, None

    # 计算单张棋盘格的R,T
    def calculate_img_rt(self, grayimg, cameraMatrix, distCoeffs):
        _, cors = self.find_corners(grayimg)

        ret, rvecs, tvecs, inliers = cv2.solvePnPRansac(
            self.objp, cors, cameraMatrix, distCoeffs)
        R, _ = cv2.Rodrigues(rvecs)

        return R, tvecs

    # 画角点
    def draw_corners(self, img: np.array, corners):
        cv2.drawChessboardCorners(
            img, (self.ROW_COR, self.COL_COR), corners, True)
