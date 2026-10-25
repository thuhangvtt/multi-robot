import numpy as np
import cv2 as cv
import glob
import sys

sys.stdout.reconfigure(encoding='utf-8')

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((9 * 6, 3), np.float32)
objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

objpoints = []
imgpoints = []

images = sorted(glob.glob('images_calibration/calibration*.jpg'))
print(f'Tim thay {len(images)} anh\n')

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    ret, corners = cv.findChessboardCorners(gray, (9, 6), None)
    print(f'  {fname}: {ret}')
    if ret:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners)
        cv.drawChessboardCorners(img, (9, 6), corners2, ret)

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