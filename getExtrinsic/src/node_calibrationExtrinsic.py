#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs import LaserScan
import threading

calibrateResultFile = 'data/cap.npz'
extrinsicFile = 'data/extrinsic.npz'
lidarData = None

lock = threading.Lock()

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img

def lidarCallback(data):
    rospy.loginfo("lidarCallback")
    global lidarData
    lock.acquire()
    lidarData = data
    lock.release()

def findChessboardInScan(ranges):
    pass

def solveExtrinsic(Pf,N):
    if Pf.shape[0] != N.shape[0]:
        raise ValueError('Pf.shape[0] != N.shape[0]:')
    A = np.zeros([Pf.shape[0], Pf.shape[1]*N.shape[1])
    A[:,0:2] = np.multiply(np.tile(Pf[:,0],(1,3)),N)
    A[:,3:5] = np.multiply(np.tile(Pf[:,1],(1,3)),N)
    A[:,6:8] = np.multiply(np.tile(Pf[:,2],(1,3)),N)
    b = np.sum(np.multiply(N, N))q
    X = np.linalg(A, b)[0]
    H = np.zeros(3,3)
    H[0,:] = X[0,2]
    H[1,:] = X[3,5]
    H[2,:] = X[6,8]

    return H

def main():
    rospy.init_node('node_calibrationExtrinsic')
    rospy.Subscriber("/scan", LaserScan, lidarCallback, queue_size=1)

    # Load previously saved data
    with np.load(calibrateResultFile) as X:
        mtx, dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((6*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
    axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)

    cap = cv2.VideoCapture(1)

    Pf = None
    N = None

    while not rospy.is_shutdown():
        ret, img = cap.read()
        if ret != True:
            cap.release()
            raise IOException('can\'t read camera')

        cv2.imshow('img',img)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (7,6), None)
        if ret:
            cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            # Find the rotation and translation vectors.
            ret,rvecs, tvecs = cv2.solvePnP(objp, corners, mtx, dist)
            # project 3D points to image plane
            imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
            img = draw(img,corners,imgpts)
            cv2.imshow('img',img)

        key = cv2.waitKey(50)
        if ret and (key & 0xFF == ord('y')):
            lock.acquire()
            ranges = lidarData.ranges
            lock.release()
            if Pf == None:
                Pf = findChessboardInScan(ranges)
            else:
                Pf = np.concatenate(Pf,findChessboardInScan(ranges))

            n = np.dot(rvecs[:,3].transpose(),tvecs) * -1 * rvecs[:,3]

            if N == None:
                N = n
            else:
                N = np.concatenate(N,n)

        elif key & 0xFF == ord('f'):
            break

    cv2.destroyAllWindows()
    H = solveExtrinsic(Pf, N)
    np.save(extrinsicFile, H)

if __name__ == '__main__':
    main()
