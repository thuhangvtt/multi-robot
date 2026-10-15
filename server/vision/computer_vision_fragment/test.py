'''
Source: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html

'''

import numpy as np
import cv2 as cv
import glob
import sys

sys.stdout.reconfigure(encoding='utf-8')

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# === SUA DOI: Loop qua tat ca anh calibration thay vi 1 anh ===
images = sorted(glob.glob('images_calibration/calibration*.jpg'))
print(f'Tim thay {len(images)} anh\n')

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (9,6), None)
    print(f'  {fname}: {ret}')
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (9,6), corners2, ret)
        # cv.imshow('img', img)
        # cv.waitKey(100)

if len(objpoints) == 0:
    print('\nKhong detect duoc anh nao!')
    exit()

print(f'\nDetect thanh cong: {len(objpoints)}/{len(images)}')

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (np.shape(img)[1], np.shape(img)[0]), 1, (np.shape(img)[1], np.shape(img)[0]))
new_image = cv.undistort(img, mtx, dist, None, newcameramtx)

print("Camera Matrix: ", mtx)
print("Distortion Coefficients: ", dist)
print(f"\nReprojection Error: {ret:.4f}")