from utils.storage import LocalStorage
from utils.calib import CalibChessboard, load_camera_param, quat_2_rot
import cv2
import numpy as np
import pickle

def test():
    TABLE_SQL_STR = '''id INTEGER PRIMARY KEY AUTOINCREMENT, 
                            rootpath text,
                            filename text, 
                            isreject bool, 
                            qw float, 
                            qx float, 
                            qy float, 
                            qz float, 
                            tx float, 
                            ty float, 
                            tz float,
                            rpje float,
                            cors blob'''
    #self.DB_FILENAME = ':memory:'
    DB_FILENAME = 'single.db'
    DB_TABLENAME = 'single'
    db = LocalStorage(DB_FILENAME)
    ret = db.create_table(DB_TABLENAME, TABLE_SQL_STR)
    db.write_data(
                DB_TABLENAME, f'null, null, null, 0, null, null, null, null, null, null, null, null, null')
    new_array = np.random.rand(88,2).astype('float32')
    array_bytes = pickle.dumps(new_array)
    db.modify_data(DB_TABLENAME, f'''SET cors=?, filename=? WHERE isreject=0 ''', (array_bytes, DB_TABLENAME))
    results = db.retrive_data(DB_TABLENAME, 'filename, cors', 'WHERE isreject=0')
    cors = [c[1] for c in results]
    r_a = pickle.loads(cors[0])
    print(np.array_equal(new_array, r_a))
    pass

def test_for_cb():
    cb = CalibChessboard(9, 12, 3.0)
    mtx, dist = load_camera_param('ELW82430-2304013_stereo_param.json')
    gray_img = cv2.imread(
        'img_left_20231228155102.jpg', 0)
    R, tvecs, cors = cb.calculate_img_rt(gray_img, mtx, dist)
    ret = cv2.undistort(gray_img, mtx, dist)
    cv2.imwrite('undist.jpg', ret)
    pass

def projection_with_distortion_by_iter(iternum):
    cb = CalibChessboard(9, 12, 3.0)
    mtx, dist = load_camera_param('ELW82430-2304013_stereo_param.json')
    gray_img = cv2.imread(
        'img_left_20231228155102.jpg', 0)
    R, tvecs, cors = cb.calculate_img_rt(gray_img, mtx, dist)

    ptw = np.asarray([6.0, 0.0, 0.0], np.float64).reshape(-1,1)

    dist_coeffs = dist.reshape(-1)

    # cam coord
    ptc = R @ ptw + tvecs
    # normalized before distortion
    x_norm = ptc[0]/ptc[2]
    y_norm = ptc[1]/ptc[2]

    x0 = x_norm.copy()
    y0 = y_norm.copy()

    for i in range(iternum):
        r2 = x_norm*x_norm + y_norm*y_norm

        distRadialA = 1/(1. + dist_coeffs[0]*r2 + dist_coeffs[1]*r2*r2 + dist_coeffs[4]*r2*r2*r2)
        distRadialB = 1.

        deltaX = 2. * dist_coeffs[2]*x_norm*y_norm + dist_coeffs[3]*(r2 + 2.*x_norm*x_norm)
        deltaY = dist_coeffs[2]*(r2 + 2.*y_norm*y_norm) + 2.*dist_coeffs[3]*x_norm*y_norm

        xCorrected = (x0 - deltaX)*distRadialA*distRadialB
        yCorrected = (y0 - deltaY)*distRadialA*distRadialB

        x_norm = xCorrected
        y_norm = yCorrected
    
    pti = mtx @ np.array([xCorrected, yCorrected, 1])
    print(pti)

def projection_with_distortion():
    cb = CalibChessboard(9, 12, 3.0)
    mtx, dist = load_camera_param('ELW82430-2304013_stereo_param.json')
    gray_img = cv2.imread(
        'img_left_20231228155102.jpg', 0)
    R, tvecs, cors = cb.calculate_img_rt(gray_img, mtx, dist)

    ptw = np.asarray([6.0, 0.0, 0.0], np.float64).reshape(-1,1)

    dist_coeffs = dist.reshape(-1)

    # cam coord
    ptc = R @ ptw + tvecs
    # normalized before distortion
    x_norm = ptc[0]/ptc[2]
    y_norm = ptc[1]/ptc[2]

    # apply distortion
    r_squared = x_norm**2 + y_norm**2
    x_distorted = x_norm*(1+dist_coeffs[0]*r_squared + dist_coeffs[1]*r_squared**2 + dist_coeffs[4]*r_squared**3) + \
        2*dist_coeffs[2]*x_norm*y_norm + dist_coeffs[3]*(r_squared + 2*x_norm**2)
    y_distorted = y_norm*(1+dist_coeffs[0]*r_squared + dist_coeffs[1]*r_squared**2 + dist_coeffs[4]*r_squared**3) + \
        dist_coeffs[2]*(r_squared + 2*y_norm**2) + 2*dist_coeffs[3]*x_norm*y_norm
    # apply the intrinsic
    pt_distorted_homo = np.array([x_distorted, y_distorted, 1])
    pixel_coord = mtx @ pt_distorted_homo
    print(pixel_coord)

def undistort_test():
    cb = CalibChessboard(9, 12, 3.0)
    mtx, dist = load_camera_param('ELW82430-2304013_stereo_param.json')
    gray_img = cv2.imread(
        'img_left_20231228155102.jpg', 0)
    R, tvecs, cors = cb.calculate_img_rt(gray_img, mtx, dist)
    # 3d pt = [6.0, 0.0, 0.0] -> 2d pixel = [1253.7157 ,  362.61615], no dist[1262.44876899, 357.54575482]
    ptw = np.asarray([6.0, 0.0, 0.0], np.float64).reshape(1,-1)
    Rt = np.hstack((R,tvecs))
    P = mtx @ Rt
    ptw_homo = np.hstack((ptw, np.ones((1,1)))).transpose()
    pt_image_homo = P @ ptw_homo 
    pt_image_normalized = pt_image_homo / pt_image_homo[-1]
    pt_image = pt_image_normalized[:2,:]

    x = pt_image[0]
    y = pt_image[1]

    # apply dist
    inv_mtx = np.linalg.inv(mtx)
    normalized = inv_mtx@pt_image_normalized
    x, y = normalized[:2]/normalized[2]
    r2 = x**2 + y**2
    radial_distortion = 1 + dist[0][0]*r2 + dist[0][1]*r2**2 + dist[0][4]*r2**3
    x_distorted = x*radial_distortion
    y_distorted = y*radial_distortion
    x_distorted += (2*dist[0][2]*x*y + dist[0][3]*(r2+2*x**2))
    y_distorted += (dist[0][2]*(r2 + 2*y**2) + 2*dist[0][3]*x*y)
    distorted_pti = mtx.dot(np.array([x_distorted, y_distorted, 1]))
    
    pass 

def projection_test():
    cb = CalibChessboard(9, 12, 3.0)
    mtx, dist = load_camera_param('ELW82430-2304013_stereo_param.json')
    gray_img = cv2.imread(
        'img_left_20231228155102.jpg', 0)
    R, tvecs, cors = cb.calculate_img_rt(gray_img, mtx, dist)
    # 3d pt = [6.0, 0.0, 0.0] -> 2d pixel = [1253.7157 ,  362.61615]
    '''
    #1 world -> camera
    ptc = R.dot(np.asarray([6.0, 0.0, 0.0], np.float64))+ tvecs.reshape(1,-1)
    #2 camera -> image without distortion
    pti = mtx.dot(ptc.reshape(-1,1))
    #3 image without distortion -> image with distortion
    pass 
    #4 image with distortion -> image with distortion
    pass
    '''
    # without distortion
    
    ptw = np.asarray([6.0, 0.0, 0.0], np.float64).reshape(1,-1)
    Rt = np.hstack((R,tvecs))
    P = mtx @ Rt
    ptw_homo = np.hstack((ptw, np.ones((1,1)))).transpose()
    pt_image_homo = P @ ptw_homo 
    pt_image_normalized = pt_image_homo / pt_image_homo[-1]
    pt_image = pt_image_normalized[:2,:]

def handeye_trans_check():
    cb = CalibChessboard(9, 12, 3.0)
    mtx, dist = load_camera_param('ELW82430-2304013_stereo_param.json')
    gray_img = cv2.imread(
        'img_left_20231228155102.jpg', 0)
    R, tvecs, cors = cb.calculate_img_rt(gray_img, mtx, dist)
    # 3d pt = [0.0, 0.0, 0.0] -> 2d pixel = [1248.3761 ,  265.9455]
    quat = np.array([0.261119412,0.762841335,-0.477854984,-0.348632134])
    R_t2b = quat_2_rot(quat)
    T_t2b = np.array([-192.19041,-162.360975,-1131.980624]).reshape(-1,1)
    Rt_t2b = np.hstack((R_t2b, T_t2b))

    ptw = np.asarray([0.0, 0.0, 0.0], np.float64).reshape(1,-1)
    ptw_homo = np.hstack((ptw, np.ones((1,1)))).transpose()
    pt_image_homo = mtx @ np.hstack((R, tvecs)) @ ptw_homo
    pt_image_normalized = pt_image_homo / pt_image_homo[-1]

    pass 


if __name__ == '__main__':
    #test_for_cb()
    undistort_test()
    handeye_trans_check()
    projection_test()
    projection_with_distortion()
    projection_with_distortion_by_iter(10000)