"""
@gothWare
"""

import numpy as np
import cv2
import glob
import yaml
#import pathlib

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((7*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:7].T.reshape(-1,2)

objpoints = [] 
imgpoints = [] 

images = glob.glob(r'/home/gautham/Documents/Codes/PyAI/Images/*.png')

found = 0
for fname in images: 
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
    ret, corners = cv2.findChessboardCorners(gray, (7,7), None)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)
        img = cv2.drawChessboardCorners(img, (7,7), corners2, ret)
        found += 1
        cv2.imshow('img', img)
        cv2.waitKey(500)


cv2.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)


data = {'camera_matrix': np.asarray(mtx).tolist(),
        'dist_coeff': np.asarray(dist).tolist()}

print("done.")

with open("calibration_matrix.yaml", "w") as f:
    yaml.dump(data, f)
