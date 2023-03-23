import numpy as np
import cv2 as cv


def calibrate_camera(images, square_size, checkerboard_dimension):
    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    h, w = checkerboard_dimension
    objpoints = []
    imgpoints = []

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(h,w,0)
    # scaled by how large the squares are
    objp = np.zeros((h*w,3), np.float32)
    objp[:,:2] = square_size * np.mgrid[0:h,0:w].T.reshape(-1,2)

    for img in images:
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (h, w), cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_EXHAUSTIVE + cv.CALIB_CB_NORMALIZE_IMAGE)
        # ret, corners = cv.findChessboardCorners(gray, (h, w), None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)
            corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)

    if objpoints:
        ret, mtx, dist, _, _ = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

        fx, fy, cx, cy = mtx[[0, 1, 0, 1], [0, 1, 2, 2]]
        k1, k2, p1, p2, k3 = dist[0]
    else:
        ret = False
        fx, fy, cx, cy = None, None, None, None
        k1, k2, p1, p2, k3 = None, None, None, None, None

    return ret, (fx, fy, cx, cy), (k1, k2, p1, p2, k3)