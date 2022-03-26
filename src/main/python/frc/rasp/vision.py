#!/usr/bin/env python3

import json
import os
import sys
import time
import math

from cscore import CameraServer, MjpegServer, UsbCamera, VideoSource
import cv2
from networktables import NetworkTablesInstance
import ntcore
import numpy as np

configFile = "/boot/frc.json"

class CameraConfig: pass

# DEBUG = False

# inRange HSV ranges for thresholding the input image for binary image
BIN_LOW_H = 60  #106 
BIN_LOW_S = 100 #168 
BIN_LOW_V = 120 #147 

BIN_HIGH_H = 90 #124 
BIN_HIGH_S = 255#255 
BIN_HIGH_V = 255#255 

EPSILON_FACTOR = 0.08 # directly influences the number of vertices for approxPolyDP

fx = 625.16 # 1875.49844 / 1920 * 640
fy = 833.57 # 1875.52991 / 1080 * 480
cam_center_x = 317.81 # 953.441373 / 1920 * 640
cam_center_y = 251.49 # 565.857930 / 1080 * 480
CAMERA_MATRIX = [[fx, 0, cam_center_x], [0, fy, cam_center_y], [0, 0, 1]]

distortion = None

# 2D projection onto the camera
OBJECT_POINTS = [
        # imperial(inches)
        [-2.5, 1.0, .0],
        [-2.5, -1.0, .0],
        [2.5, 1.0, .0],
        [2.5, -1.0, .0],
        ]

# convert to numpy array
CAMERA_MATRIX, OBJECT_POINTS = [np.array(x) for x in [CAMERA_MATRIX, OBJECT_POINTS]]

camAngle2Shooter = 30

rMatShooter = [
        [1.0, 0.0, 0.0],
        [0.0, math.cos(camAngle2Shooter / 180 * math.pi), -math.sin(camAngle2Shooter / 180 * math.pi)],
        [0.0, math.sin(camAngle2Shooter / 180 * math.pi), math.cos(camAngle2Shooter / 180 * math.pi)],
        ]

tMatShooter = [
        [0.0],
        [5.25],
        [7.5]
        ]

team = None
server = False
cameraConfigs = []
switchedCameraConfigs = []
cameras = []

with open('/boot/frc.json') as f:
    config2 = json.load(f)
width = config2['cameras'][0]['width']
height = config2['cameras'][0]['height']

 # start NetworkTables
ntinst = NetworkTablesInstance.getDefault()
vision_nt = ntinst.getTable('Vision')


def parseError(str):
    """Report parse error."""
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

def readCameraConfig(config):
    """Read single camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    # stream properties
    cam.streamConfig = config.get("stream")

    cam.config = config

    cameraConfigs.append(cam)
    return True

def readSwitchedCameraConfig(config):
    """Read single switched camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read switched camera name")
        return False

    # path
    try:
        cam.key = config["key"]
    except KeyError:
        parseError("switched camera '{}': could not read key".format(cam.name))
        return False

    switchedCameraConfigs.append(cam)
    return True

def readConfig():
    """Read configuration file."""
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        if not readCameraConfig(camera):
            return False

    # switched cameras
    if "switched cameras" in j:
        for camera in j["switched cameras"]:
            if not readSwitchedCameraConfig(camera):
                return False

    return True

def startCamera(config):
    """Start running the camera."""
    print("Starting camera '{}' on {}".format(config.name, config.path))
    inst = CameraServer.getInstance()

    camera = UsbCamera(config.name, config.path)
    server = inst.startAutomaticCapture(camera=camera, return_server=True)


    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)

    if config.streamConfig is not None:
        server.setConfigJson(json.dumps(config.streamConfig))

    return camera

def startSwitchedCamera(config):
    """Start running the switched camera."""
    print("Starting switched camera '{}' on {}".format(config.name, config.key))
    server = CameraServer.getInstance().addSwitchedCamera(config.name)

    def listener(fromobj, key, value, isNew):
        if isinstance(value, float):
            i = int(value)
        if i >= 0 and i < len(cameras):
            server.setSource(cameras[i])
        elif isinstance(value, str):
            for i in range(len(cameraConfigs)):
                if value == cameraConfigs[i].name:
                    server.setSource(cameras[i])
                    break

    NetworkTablesInstance.getDefault().getEntry(config.key).addListener(
            listener,
            ntcore.constants.NT_NOTIFY_IMMEDIATE |
            ntcore.constants.NT_NOTIFY_NEW |
            ntcore.constants.NT_NOTIFY_UPDATE)

    return server

def setupCameras():
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

     # read configuration
    if not readConfig():
        sys.exit(1)

    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for Team {}".format(team))
        ntinst.startClientTeam(team)

     # start cameras
    for config in cameraConfigs:
        cameras.append(startCamera(config))

     # start switched cameras
    for config in switchedCameraConfigs:
        startSwitchedCamera(config)

    os.system("v4l2-ctl -c auto_exposure=1")
    os.system("v4l2-ctl -c exposure_time_absolute=20")

# def framePerSecond(start_time):
#     processing_time = time.time() - start_time
#     fps = 1 / processing_time
#     return fps

# return a list of all the vertices from the binary image
def verticesFromBin(binary_img, output_img):
    _, contour_list, _ = cv2.findContours(binary_img, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_TC89_L1)

    vertice_list = []
    for contour in contour_list:
        # Ignore small contours that could be because of noise/bad thresholding
        if cv2.contourArea(contour) < 25:
            continue

        epsilon = EPSILON_FACTOR * cv2.arcLength(contour, True)
        quadrilateral = cv2.approxPolyDP(contour, epsilon, True)
        vertices = quadrilateral.ravel().tolist() # 4 corners of the quadrilateral
        vertice_list.append(vertices)

        # Draw rectangle
        cv2.drawContours(output_img, [quadrilateral], -1, color = (0, 0, 255), thickness = 2)
        for point in contour:
            # print(point)
            # print(tuple(point[0]))
            cv2.circle(output_img, center=tuple(point[0]), radius = 1, color = (0,0,255), thickness = 1)
    return vertice_list

# shows the HSV value at the center pixel of the camera frame
def getCenterHSV(output_img):
    # for tuning only
    pixel_center = hsv_img[int(height/2), int(width/2)]
    cv2.circle(output_img, center = (int(width/2), int(height/2)), radius = 1, color = (0, 0, 255), thickness = 1)
    cv2.putText(output_img, "HSV: " + str(pixel_center), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))

# sort the list of quadrilaterals by sorting x first (left to right), and sort y (up then down). The order looks like letter "N"
def sortQuadList(quad_list):
    if len(quad_list) == 8:
        # sort according to x
        points = list(zip(quad_list[::2], quad_list[1::2]))
        points.sort(key=lambda t: t[0])
        # sort according to y
        y = lambda p: p[1]
        quad_list.clear()
        for pair in zip(points[::2], points[1::2]):
            quad_list.extend([val for point in sorted(pair, key=y) for val in point])

# calculate the translation vector from the shoooter to the center
def calcShooterToCenter(rvec, tvec):
    bottom_list = [[0.0, 0.0, 0.0, 1.0]]

    # cosntructing a transformation matrix from camera to hoop center
    trans_matrix = np.hstack((rvec, tvec))
    trans_matrix = np.vstack((trans_matrix, bottom_list))

    # cosntructing a transformation matrix from shooter to camera
    shooter_to_cam_matrix = np.hstack((rMatShooter, tMatShooter))
    shooter_to_cam_matrix = np.vstack((shooter_to_cam_matrix, bottom_list))

    center_to_tape = [[.0],[.0],[-27.0], [1.0]]
    center = trans_matrix.dot(center_to_tape)
    center = shooter_to_cam_matrix.dot(center)
    center = center.tolist()

    return center

# def calcHoopCenter(rvec, tvec):
#     bottom_list = [[0.0, 0.0, 0.0, 1.0]]
#     trans_matrix = np.hstack((rvec, tvec))
#     trans_matrix = np.vstack((trans_matrix, bottom_list))
#     center_to_tape = [[.0],[.0],[-27.0], [1.0]]
#     center = trans_matrix.dot(center_to_tape)
#     center = center.tolist()
#     return center

# group points from the approxpolydp 
def makeImagePoints(vertice_list):
    image_points = []
    # sort image points
    idx = 0
    for quad in vertice_list:
        if len(quad) == 8: # TODO: put it up a level
            image_points.append([])
            sortQuadList(quad)

            result = list(map(list, zip(quad[::2], quad[1::2])))
            for i in result:
                i = list(map(float, i))
                image_points[idx].append(i)
            idx+=1
    return image_points

# returns a list of hoop centers by using groups of quadrilaterals as image points
def getHoopCenter(output_img, vertice_list):
    # focal lengths
    image_points = makeImagePoints(vertice_list)
    hoop_coords = []
    idx = 0
    for quad_points in image_points:
    # if len(image_points) == len(OBJECT_POINTS):
        quad_points = [np.array(x) for x in [quad_points ]] # convert to numpy array
        _, rvec, tvec = cv2.solvePnP(OBJECT_POINTS, quad_points[0], CAMERA_MATRIX, distortion)#, flags=cv2.SOLVEPNP_EPNP)
        rvec, _ = cv2.Rodrigues(rvec)
        center = calcShooterToCenter(rvec, tvec)
        hoop_coords.append([])
        for i in range(3):
            hoop_coords[idx].append(int(center[i][0]))
        idx+=1

    # show corners and coords
    # cv2.putText(output_img, str(hoop_coords), (200, 120), 0, 3, (128, 255, 0), 3)
    for quad in image_points:
        for i in quad:
            cv2.circle(output_img, center = tuple(list(map(int, i))), radius = 4, color = (255, 0, 255), thickness = -1)
    
    return hoop_coords

# rejects outliers in an array judging from each point's difference to the median
def rejectOutliers(data, m = 2):
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d/mdev if mdev else 0.
    return data[s<m]

def reduceStdDev(coords):
    coords = rejectOutliers(coords)
    sigma = np.std(coords)
    # mean = np.mean(coords)
    median = np.median(coords)
    if (sigma < 30):
        return median
    else:
        return "NaN"

# Chooses the best center point out of all generated from each set of image points
# returns NaN if value too off or none detected
def getBestCenter(hoop_centers):
    coords = [np.array(x) for x in [hoop_centers]]
    coords = coords[0]
    if len(hoop_centers) > 0:
        return [reduceStdDev(coords[:,0]), reduceStdDev(coords[:,1]), reduceStdDev(coords[:,2])]
    return ["NaN", "NaN", "NaN"]

if __name__ == "__main__":
    # set up cameras according to the config
    setupCameras()
    img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
    inst = CameraServer.getInstance()
    input_stream = inst.getVideo()
    output_stream = inst.putVideo('Processed', width, height)
    binary_output_stream = inst.putVideo('Binary', width, height)

    while True:
        # start_time = time.time()
        frame_time, input_img = input_stream.grabFrame(img)

        # Notify output of error and skip iteration
        if frame_time == 0:
            output_stream.notifyError(input_stream.getError())
            binary_output_stream.notifyError(input_stream.getError())
            continue

        # Convert to HSV and threshold image
        # input_img = cv2.GaussianBlur(input_img, (3,3), cv2.BORDER_CONSTANT)
        hsv_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)
        binary_img = cv2.inRange(hsv_img, (BIN_LOW_H, BIN_LOW_S, BIN_LOW_V), (BIN_HIGH_H, BIN_HIGH_S, BIN_HIGH_V))
        output_img = np.copy(input_img)

        # getCenterHSV(output_img) # for tuning binary image HSV range

        # get the center of the hoop and upload it to the network table
        vertice_list = verticesFromBin(binary_img, output_img)
        hoop_centers = getHoopCenter(output_img, vertice_list)
        center = getBestCenter(hoop_centers)

        cv2.putText(output_img, str(center), (167, 40), 0, (1), (128, 255, 0), 1)
        vision_nt.putNumberArray('translation_vector', center)

        # display the frames 
        output_stream.putFrame(output_img) # on port 1182
        binary_output_stream.putFrame(binary_img) # port 1183

