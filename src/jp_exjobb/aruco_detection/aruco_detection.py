from scipy.spatial.transform import Rotation as R

import numpy as np
import cv2 as cv


def undistort(xd, yd, calib, distortion, maxiter=1000, eps=1e-4):
    fx, fy, cx, cy = calib
    k1, k2, p1, p2, k3 = distortion

    xorig, yorig = xd, yd

    xd = (xd - cx) / fx
    yd = (yd - cy) / fy
    x, y, = xd, yd

    """
    The L515 camera follows a Brown-Conrady distortion model
    which uses the formula:
        x_distorted = x + x*(k1*r^2 + k2*r^4 + k3*r^6) + p1*(r^2 + 2x^2) + p2*2xy
        y_distorted = y + y*(k1*r^2 + k2*r^4 + k3*r^6) + p1*2xy          + p2*(r^2 + 2y^2)
    where
        r = sqrt(x^2 + y^2)

    We solve for x by fixed point iteration (same for y, but I'll skip writing out both...)
    by setting
        x_distorted = x + d(x)
    where
        d(x) = x*(k1*r^2 + k2*r^4 + k3*r^6) + p1*(r^2 + 2x^2) + p2*2xy .
    Then we can reformulate the problem as
        x = x_distorted - d(x) ,
    finally we define
        f(x) = x_distorted - d(x)
    and we have a fixed-point iteration problem:
        x = f(x) .

    OpenCV also solves undistortion by fixed-point iteration, but they use a slightly different distortion model.
    """

    for _ in range(maxiter):
        r2 = x ** 2 + y ** 2
        rad_factor = r2 * (k1 + r2 * (k2 + r2 * k3))
        xy2 = 2.0 * x * y
        r2xx = r2 + 2 * x * x
        r2yy = r2 + 2 * y * y
        dx = x * rad_factor + p1 * r2xx + p2 * xy2
        dy = y * rad_factor + p1 * xy2 + p2 * r2yy

        xproj = (x + dx) * fx + cx
        yproj = (y + dy) * fy + cy
        error = np.sqrt((xorig - xproj) ** 2 + (yorig - yproj) ** 2)
        if error < eps: break

        x = xd - dx
        y = yd - dy

    x = x * fx + cx
    y = y * fy + cy

    return x, y


def cam_coords2pixel_coords(p, calib):
    """
        Converts coordinates in the camera coordinate frame to pixel coordinates. The
        parameters are assumed to be,

            p = [x, y, z],
            calib = (fx, fy, cx, cy),

        where p is a vector with the 3d coordinates and calib contains the camera
        calibration parameters.
    """
    fx, fy, cx, cy = calib
    x = int(round((fx * p[0] / p[2]) + cx))
    y = int(round((fy * p[1] / p[2]) + cy))
    return x, y


def image_coordinate_system(img, origin, tangents, calib, scale):
    orig_x, orig_y = cam_coords2pixel_coords(origin, calib)
    print(orig_x, orig_y)
    cv.circle(img, (orig_x, orig_y), 3, (0, 255, 0))

    axis_col = [
        (0, 0, 255),
        (0, 255, 0),
        (255, 0, 0)
    ]

    for i, t in enumerate(tangents):
        print(np.linalg.norm(scale * t))
        v = origin + scale * t
        x, y = cam_coords2pixel_coords(v, calib)
        cv.line(img, (orig_x, orig_y), (x, y), axis_col[i], 2)


def aruco_detection(rgb, calib, distortion, aruco_ids:dict):
    """Detect ARUCO codes.

    Args:
        rgb (Matrix): Image pixels in RGB.
        calib (Tuple): Values for camera calibrations parameters
                       (fx, fy, cx, cy).
        distortion (Tuple): Distortion values (k1, k2, p1, p2, k3).
        aruco_ids (Dict): Containing the ARUCO ids to detect with fromat
                          {id: (size, length)}.

    Returns:
        Dict[int: List[Tuple(int, List[float], List[float])]]: 
               Dict of id (key) and list (value) containing a Tuple with
               (aruco_size, list of quaternion, list of translation values [x, y, z])
    """
    # TODO: Consider not computing these dictionaries everytime.
    # Aruco size with expected ids for that
    aruco_size_ids = {marker_size: set([id for (id, size), _ in aruco_ids.items()
                          if size == marker_size]) 
                      for marker_size in range(4, 8)}

    # Aruco size with expected id and its corresponding length.
    aruco_length_ids = {marker_size: {id: length for (id, size), length in aruco_ids.items()
                                      if size == marker_size} 
                       for marker_size in range(4, 8)}

    # Dict of size and matching aruco dict to detect.
    ds = {4: cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_1000),
          5: cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_1000),
          6: cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_1000),
          7: cv.aruco.getPredefinedDictionary(cv.aruco.DICT_7X7_1000)}

    # p = cv.aruco.DetectorParameters_create()
    # d = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_7X7_100)  # Opencv 4.7
    p = cv.aruco.DetectorParameters()

    K = np.zeros((3, 3))
    # Create camera matrix
    # camera matrix (K):
    #            fx  0  cx
    #            0  fy  cy
    #            0  0   1
    fx, fy, cx, cy = calib
    K[0, 0] = fx
    K[1, 1] = fy
    K[0, 2] = cx
    K[1, 2] = cy
    K[2, 2] = 1

    detected_ids = dict()

    for size, aruco_dict in ds.items():
        # Detect all aruco markers in the image and their ids
        corners, ids, _ = cv.aruco.detectMarkers(rgb, aruco_dict, parameters=p)

        if len(corners) == 0:
            continue

        ids = ids.reshape(-1) #  Reshape because [[1],
                              #                   [2]]

        for i, (c, id) in enumerate(zip(corners, ids)):
            # Skip ids that are not sought after with specified size.
            if id not in aruco_size_ids[size]:
                continue

            # print("ID = %s" % id)
            coords = np.zeros((c.shape[1], 2))
            for j in range(coords.shape[0]):
                coords[j] = undistort(c[0, j, 0], c[0, j, 1], calib, distortion)
                # coords[j] = c[0, j]

            length_of_id = aruco_length_ids[size][id]
            l = length_of_id / 10**3 #  Convert length from (mm) to SI-unit (m).
            obj_points = np.array([	[l, 0, 0],
                        [l, l, 0],
                        [0, l, 0],
                        [0, 0, 0]], dtype=float)

            _, r, t = cv.solvePnP(obj_points, coords, K, None)
            # print("=" * 10 + "SolvePnP" + "=" * 10)
            # divider = '-' * 10
            # print("Axis angle:\n%s\n%sTranslation Vector:\n%s\n%s" % (r,divider,t, divider))

            # Get rotation matrix
            rotation_matrix = cv.Rodrigues(r)
            # print("Rotation vector (Rodrigues):\n%s\n%s" % (rotation_matrix[0], divider))
            quaternion = R.from_matrix(rotation_matrix[0]).as_quat()
            # print("Quaternion vector:\n%s\n%s" % (quaternion, divider))

            detected_ids[(id, size)] = (t.reshape(-1), quaternion)
            # cv.drawFrameAxes(rgb, K, distortion, r, t, 0.1)

    return detected_ids
