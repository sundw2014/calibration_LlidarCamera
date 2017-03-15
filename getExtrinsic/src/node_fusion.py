#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import LaserScan
import threading
import math

calibrateResultFile = 'data/cap.npz'
extrinsicFile = 'data/extrinsic.npy'
lidarData = None

lock = threading.Lock()

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img

def getLidarPoints(li_data, H):
    ranges = li_data.ranges
    angularResolution = 270.0/811.0
    lidarPoints = None
    for index, r in enumerate(ranges):
        # print angularResolution
        if r >= li_data.range_max or r <= li_data.range_min:
            continue
        Pf = np.array(RTheta2XY(r, index*angularResolution/180.0*math.pi)).reshape(3,1)
        P = (H.dot(Pf)).reshape(1,3)
        if lidarPoints == None:
            lidarPoints = P
        else:
            lidarPoints = np.concatenate((lidarPoints,P))

    return lidarPoints

def drawPoints(img, imgpts):
    for p in imgpts:
        try:
            cv2.circle(img, tuple(p[0].astype(int)), 3, (255,0,0))
        except OverflowError as e:
            pass
    return img

def lidarCallback(data):
    # rospy.loginfo("lidarCallback")
    global lidarData
    lock.acquire()
    lidarData = data
    lock.release()

def RTheta2XY(r, theta):
    x = r * math.cos(theta)
    y = r * math.sin(theta)
    # print theta, math.sin(theta)
    # print [x,y,1]
    return [x,y,1]

def main():
    rospy.init_node('node_calibrationExtrinsic')
    rospy.Subscriber("/scan", LaserScan, lidarCallback, queue_size=1)

    # Load previously saved data
    with np.load(calibrateResultFile) as X:
        mtx, dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]

    # Load previously saved data
    H = np.load(extrinsicFile)

    # criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # objp = np.zeros((6*7,3), np.float32)
    # objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
    # axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
    #
    cap = cv2.VideoCapture(1)

    while not rospy.is_shutdown():
        ret, img = cap.read()
        if ret != True:
            cap.release()
            raise IOException('can\'t read camera')

        # cv2.imshow('img',img)
        # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # # Find the chess board corners
        # ret, corners = cv2.findChessboardCorners(gray, (7,6), None)
        # if ret:
        #     cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        #     # Find the rotation and translation vectors.
        #     ret,rvecs, tvecs = cv2.solvePnP(objp, corners, mtx, dist)
        #     # project 3D points to image plane
        #     imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
        #     img = draw(img,corners,imgpts)
        #     cv2.imshow('img',img)

        lock.acquire()
        li_data = lidarData
        lock.release()
        # draw lidar point on image
        lidarPoints = getLidarPoints(li_data, H)
        print lidarPoints
        imgpts, jac = cv2.projectPoints(lidarPoints, np.zeros([1,3]), np.zeros([1,3]), mtx, dist)
        print imgpts
        img = drawPoints(img, imgpts)
        cv2.imshow('img',img)
        key = cv2.waitKey(50)
        if key & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
