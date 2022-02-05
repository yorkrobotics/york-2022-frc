#!/usr/bin/env python3
#----------------------------------------------------------------------------
# Copyright (c) 2018 FIRST. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.
#----------------------------------------------------------------------------

import json
import time
import sys
import cv2
import numpy as np
import os

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from networktables import NetworkTablesInstance
import ntcore

#    JSON format:
#    {
        #         "team": <team number>,
        #         "ntmode": <"client" or "server", "client" if unspecified>
        #         "cameras": [
            #              {
                #                    "name": <camera name>
                #                    "path": <path, e.g. "/dev/video0">
                #                    "pixel format": <"MJPEG", "YUYV", etc>    // optional
                #                    "width": <video mode width>                  // optional
                #                    "height": <video mode height>                // optional
                #                    "fps": <video mode fps>                        // optional
                #                    "brightness": <percentage brightness>     // optional
                #                    "white balance": <"auto", "hold", value> // optional
                #                    "exposure": <"auto", "hold", value>        // optional
                #                    "properties": [                                  // optional
                    #                         {
                        #                              "name": <property name>
                        #                              "value": <property value>
                        #                         }
                    #                    ],
                #                    "stream": {                                        // optional
                    #                         "properties": [
                        #                              {
                            #                                    "name": <stream property name>
                            #                                    "value": <stream property value>
                            #                              }
                        #                         ]
                    #                    }
                #              }
            #         ]
        #         "switched cameras": [
            #              {
                #                    "name": <virtual camera name>
                #                    "key": <network table key used for selection>
                #                    // if NT value is a string, it's treated as a name
                #                    // if NT value is a double, it's treated as an integer index
                #              }
            #         ]
        #    }

configFile = "/boot/frc.json"

class CameraConfig: pass

team = None
server = False
cameraConfigs = []
switchedCameraConfigs = []
cameras = []

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

def getCenterHSV(output_img):
    # for testing purposes 
    pixel_center = hsv_img[int(height/2), int(width/2)]
    cv2.circle(output_img, center = (int(width/2), int(height/2)), radius = 1, color = (0, 0, 255), thickness = 1)
    cv2.putText(output_img, "HSV: " + str(pixel_center), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))

def sortQuadList(quad_list):
    # sort according to x
    print("[DEBUG] old quad:", quad_list)
    if len(quad_list) == 8:
        for j in range(0, len(quad_list) - 2, 2):
            max = quad_list[j]
            for i in range(j, len(quad_list), 2):
                if quad_list[i] > max: # if current x value is the biggest
                    max = quad_list[j]
                    quad_list.insert(j, quad_list.pop(i)) # insert x coord
                    quad_list.insert(j+ 1, quad_list.pop(i+1)) # insert y coord
        # sort according to y
        if quad_list[1] < quad_list[3]:
            quad_list[2], quad_list[0] = quad_list[0], quad_list[2]
            quad_list[3], quad_list[1] = quad_list[1], quad_list[3]
        if quad_list[5] < quad_list[7]:
            quad_list[6], quad_list[4] = quad_list[4], quad_list[6]
            quad_list[7], quad_list[5] = quad_list[5], quad_list[7]


def getHoopCenter(output_img, vertice_list):
    # focal lengths
    fx = 2022.3166
    fy = 2023.5474
    width2 = 892.06
    height2 = 512.88
    camera_matrix = [[fx, 0, width2], [0, fy, height2], [0, 0, 1]]
    # 2D projection onto the camera
    object_points = [
            # [  -13.72 ,   -1.0 ,   15.90 ],
            # [  -13.72 ,   1.0 ,   15.90 ],
            # [  -12.25 ,   1.0 ,   17.05 ],
            # [  -12.25 ,   -1.0 ,   17.05 ],

            # [ -7.5,  -1.0,   19.618 ],
            # [ -7.5,  1.0,   19.618 ],
            # [-2.7,  1.0,   20.825 ],
            # [-2.7,  -1.0,   20.825 ],

            # [2.7,  -1.0,   20.825 ],
            # [2.7,  1.0,   20.825 ],
            # [7.5,  1.0,   19.618 ],
            # [7.5,  -1.0,   19.618 ],

            # [13.72 ,   -1.0 ,   15.90 ],
            # [13.72 ,   1.0 ,   15.90 ],
            # [12.25 ,   1.0 ,   17.05 ],
            # [12.25 ,   -1.0 ,   17.05 ],



            # [-330.0, -30.0, .0],
            # [-330.0, 30.0, .0],
            # [-200.0, 30.0, .0],
            # [-200.0, -30.0, .0],

            # [-60.0, -30.0, .0],
            # [60.0, 30.0, .0],
            # [60.0, 30.0, .0],
            # [-60.0, -30.0, .0],

            # [200.0, -30.0, .0],
            # [200.0, 30.0, .0],
            # [330.0, 30.0, .0],
            # [330.0, -30.0, .0],

            # imperial(inches)
            [-12.9, -1.0, .0],
            [-12.9, 1.0, .0],
            [-7.9, 1.0, .0],
            [-7.9, -1.0, .0],

            [-2.5, -1.0, .0],
            [2.5, 1.0, .0],
            [2.5, 1.0, .0],
            [-2.5, -1.0, .0],

            [7.9, -1.0, .0],
            [7.9, 1.0, .0],
            [12.9, 1.0, .0],
            [12.9, -1.0, .0],
            ]

    # distortion = [1.803, -186.72, 0.0, 0.0, 6469.52]
    distortion = None


    image_points = []

    # sort image points
    for quad in vertice_list:
        sortQuadList(quad)
        print("\n[DEBUG] Sorted Quad List: ", quad)

        result = list(map(list, zip(quad[::2], quad[1::2])))
        for i in result:
            i = list(map(float, i))
            image_points.append(i)


    # for l in vertice_list:
    #     result = list(map(list, zip(l[::2], l[1::2])))
    #     for i in result:
    #         i = list(map(float, i))
    #         image_points.append(i)


    hoop_coord = []
    if len(image_points) == 12:
        # camera_matrix, object_points, image_points, distortion = [np.array(x) for x in [camera_matrix, object_points, image_points, distortion]]
        camera_matrix, object_points, image_points = [np.array(x) for x in [camera_matrix, object_points, image_points]]

        ret, rvec, T = cv2.solvePnP(object_points, image_points, camera_matrix, distortion, flags=cv2.SOLVEPNP_EPNP)
        R, _ = cv2.Rodrigues(rvec)

        for i in T:
            hoop_coord.append(int(i))

        cv2.putText(output_img, str(hoop_coord), (200, 100), 0, 3, (128, 255, 0), 3)

    print("[DEBUG] Hoop center: ", hoop_coord)

    # print("image_points:", image_points)
    # draw points to debug
    # print(hoop_coord)

    for i in image_points:
        cv2.circle(output_img, center = tuple(list(map(int, i))), radius = 4, color = (255, 0, 255), thickness = -1)
    return hoop_coord



if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

     # read configuration
    if not readConfig():
        sys.exit(1)


    with open('/boot/frc.json') as f:
        config = json.load(f)
    camera = config['cameras'][0]

    width = camera['width']
    height = camera['height']

     # start NetworkTables
    ntinst = NetworkTablesInstance.getDefault()

    vision_nt = ntinst.getTable('Vision')
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


    img = np.zeros(shape=(240, 320, 3), dtype=np.uint8)

    with open('/boot/frc.json') as f:
        config2 = json.load(f)

    width = config2['cameras'][0]['width']
    height = config2['cameras'][0]['height']

    inst = CameraServer.getInstance()
    input_stream = inst.getVideo()
    output_stream = inst.putVideo('Processed', width, height)
    binary_output_stream = inst.putVideo('Binary', width, height)

    os.system("echo hello world!")
    os.system("v4l2-ctl -c auto_exposure=1")
    os.system("v4l2-ctl -c exposure_time_absolute=30")
     # loop forever
    while True:
        start_time = time.time()

        frame_time, input_img = input_stream.grabFrame(img)
        output_img = np.copy(input_img)

        # Notify output of error and skip iteration
        if frame_time == 0:
            output_stream.notifyError(input_stream.getError())
            binary_output_stream.notifyError(input_stream.getError())
            continue

        # Convert to HSV and threshold image
        hsv_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)

        binary_img = cv2.inRange(hsv_img, (75, 100, 120), (95, 255, 255))
        ret,thresh = cv2.threshold(binary_img, 127, 255, 0)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10,10))
        # binary_img = cv2.morphologyEx(binary_img, cv2.MORPH_CLOSE, kernel)
        # binary_img = cv2.dilate(thresh, kernel, iterations=3)

        # getCenterHSV(output_img)

        _, contour_list, _ = cv2.findContours(binary_img, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)

        vertice_list = []
        for contour in contour_list:
            # Ignore small contours that could be because of noise/bad thresholding
            if cv2.contourArea(contour) < 15:
                continue
            # cv2.drawContours(output_img, contour, -1, color = (255, 255, 255), thickness = -1)

            epsilon = 0.1 * cv2.arcLength(contour, True)
            quadrilateral = cv2.approxPolyDP(contour, epsilon, True)
            vertices = quadrilateral.ravel()

            cv2.drawContours(binary_img, [quadrilateral], 0, (128, 255, 255), 3)
            vertice_list.append(quadrilateral.ravel().tolist())

            # Draw rectangle and circle
            cv2.drawContours(output_img, [quadrilateral], -1, color = (0, 0, 255), thickness = 2)
            #cv2.circle(output_img, center = center, radius = 3, color = (0, 0, 255), thickness = -1)

        vision_nt.putNumberArray('translation_vector', getHoopCenter(output_img, vertice_list))

        processing_time = time.time() - start_time
        fps = 1 / processing_time

        binary_output_stream.putFrame(binary_img)
        output_stream.putFrame(output_img)
        time.sleep(processing_time)

