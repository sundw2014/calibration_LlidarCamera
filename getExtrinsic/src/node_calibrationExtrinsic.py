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
    cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 1)
    cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 1)
    cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 1)
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

def findChessboardInScan(li_data):
    ranges = li_data.ranges
    for index, r in enumerate(ranges):
        angularResolution = 270.0/811.0
        # print angularResolution
        if r >= li_data.range_max or r <= li_data.range_min:
            continue
        if r < 1.3:
            print 'ang = ', index*angularResolution, ', range:',ranges[index+3:index+11]
            # print ranges[index+3], ranges[index+5]
            A = np.array(RTheta2XY(ranges[index+3], (index+3)*angularResolution/180.0*math.pi)).reshape(1,3)
            A = np.concatenate((A, np.array(RTheta2XY(ranges[index+10], (index+10)*angularResolution/180.0*math.pi)).reshape(1,3)))
            return A
    return None

def solveExtrinsic(Pf,N):
    print 'Pf', Pf
    print 'N', N
    if Pf.shape[0] != N.shape[0]:
        raise ValueError('Pf.shape[0] != N.shape[0]:')
    A = np.zeros([Pf.shape[0], Pf.shape[1]*N.shape[1]])
    A[:,0:3] = np.multiply(np.tile(Pf[:,0],(3,1)).transpose(),N)
    A[:,3:6] = np.multiply(np.tile(Pf[:,1],(3,1)).transpose(),N)
    A[:,6:9] = np.multiply(np.tile(Pf[:,2],(3,1)).transpose(),N)
    b = np.sum(np.multiply(N, N),1)
    X = np.linalg.lstsq(A, b)[0]
    H = np.zeros([3,3])
    H[0,:] = X[0:3]
    H[1,:] = X[3:6]
    H[2,:] = X[6:9]

    return H.transpose()

def main():
    rospy.init_node('node_calibrationExtrinsic')
    rospy.Subscriber("/scan", LaserScan, lidarCallback, queue_size=1)

    # Load previously saved data
    with np.load(calibrateResultFile) as X:
        mtx, dist = [X[i] for i in ('mtx','dist')]

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((6*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
    axis = np.float32([[13,0,0], [0,13,0], [0,0,-13]]).reshape(-1,3)

    cap = cv2.VideoCapture(1)

    Pf = None
    N = None
    Rs = list()
    Ts = list()
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
            li_data = lidarData
            lock.release()
            if Pf == None:
                Pf = findChessboardInScan(li_data)
            else:
                Pf = np.concatenate((Pf, findChessboardInScan(li_data)))
            # print 'Pf:',Pf

            rmat,_ = cv2.Rodrigues(rvecs)
            n = np.dot(rmat[:,2],tvecs) * -1 * rmat[:,2]
            n = n.reshape(1,3)
            n = np.tile(n,(2, 1))

            if N == None:
                N = n
            else:
                N = np.concatenate((N,n))
            Rs.append(rvecs)
            Ts.append(tvecs)
            # print N
        elif key & 0xFF == ord('f'):
            break

    cv2.destroyAllWindows()
    np.savez('data/tmp.npz',Pf=Pf,N=N,Rs=np.array(Rs),Ts=np.array(Ts))
    H = solveExtrinsic(Pf, N)
    np.save(extrinsicFile, H)

if __name__ == '__main__':
    main()
