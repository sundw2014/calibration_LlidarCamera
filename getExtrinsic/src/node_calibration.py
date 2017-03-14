#!/usr/bin/env python
import numpy as np
import cv2
# import glob

calibrateResultFile = 'data/cap.npz'

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

cap = cv2.VideoCapture(1)

Poses = 10
count = 0
while True:
    global count
    ret, img = cap.read()
    cv2.imshow('img', img)
    k = cv2.waitKey(50)
    if ord('f') == k&0xFF:
        break

    if ret != True:
        cap.release()
        raise IOException('can\'t read camera')
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,6), None)
    if not ret:
        continue

    count = count + 1
    if count > Poses:
        break
    objpoints.append(objp)
    corners2=cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
    imgpoints.append(corners)
    # Draw and display the corners
    cv2.drawChessboardCorners(img, (7,6), corners, ret)
    cv2.imshow('img', img)
    cv2.waitKey(1500)

cv2.destroyAllWindows()
cap.release()

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
np.savez(calibrateResultFile, mtx= mtx, dist = dist, rvecs = rvecs, tvecs = tvecs)
