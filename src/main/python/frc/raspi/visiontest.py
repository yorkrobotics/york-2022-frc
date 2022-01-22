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

def getDistance(center_list):
    # assuming only three values in center_list
    if len(center_list) == 3:
        left_point_x = center_list[0][0] # could be 1 for the second index
        #mid_point_x = center_list[1][0]
        right_point_x = center_list[2][0]
        
        diff = right_point_x - left_point_x

        focal_length = 360
        distance = 9 * focal_length / diff

        return distance
    return -1

def getFocalLength(pixel_length):
    if pixel_length != -1:
        actual_tape_width = 9 # inches
        actual_distance = 76 # inches
        focal_length = pixel_length * actual_distance / actual_tape_width
        return focal_length # 183 or 360
    return -1

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
        binary_img = cv2.inRange(hsv_img, (75, 49, 200), (90, 255, 255))

        # for testing purposes 
        pixel_center = hsv_img[int(height/2), int(width/2)]
        hue = pixel_center[0]
        sat = pixel_center[1]
        val = pixel_center[2]

        binary_output_stream.putFrame(binary_img)

        _, contour_list, _ = cv2.findContours(binary_img, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)

        x_list = []
        y_list = []

        global vertices

        center_list = []
        for contour in contour_list:

            # Ignore small contours that could be because of noise/bad thresholding
            if cv2.contourArea(contour) < 15:
                continue

            cv2.drawContours(output_img, contour, -1, color = (255, 255, 255), thickness = -1)

            rect = cv2.minAreaRect(contour)
            center, size, angle = rect
            center = tuple([int(dim) for dim in center]) # Convert to int so we can draw

            center_list.append(center)

            box = cv2.boxPoints(rect)
            box = np.int0(box)
            left_point_x = np.min(box[:, 0])
            right_point_x = np.max(box[:, 0])
            top_point_y = np.min(box[:, 1])
            bottom_point_y = np.max(box[:, 1])

            left_point_y = box[:, 1][np.where(box[:, 0] == left_point_x)][0]
            right_point_y = box[:, 1][np.where(box[:, 0] == right_point_x)][0]
            top_point_x = box[:, 0][np.where(box[:, 1] == top_point_y)][0]
            bottom_point_x = box[:, 0][np.where(box[:, 1] == bottom_point_y)][0]
            vertices = np.array([[top_point_x, top_point_y], [bottom_point_x, bottom_point_y], [left_point_x, left_point_y], [right_point_x, right_point_y]])
            # Draw rectangle and circle
            cv2.drawContours(output_img, [box], -1, color = (0, 0, 255), thickness = 2)
            cv2.circle(output_img, center = center, radius = 3, color = (0, 0, 255), thickness = -1)

            x_list.append((center[0] - width / 2) / (width / 2))
            x_list.append((center[1] - width / 2) / (width / 2))

        cv2.circle(output_img, center = (int(width/2), int(height/2)), radius = 1, color = (0, 0, 255), thickness = 1)

        vision_nt.putNumberArray('target_x', x_list)
        vision_nt.putNumberArray('target_y', y_list)

        processing_time = time.time() - start_time
        fps = 1 / processing_time
        #cv2.putText(output_img, "HSV: " + str(pixel_center) + "coord: " + str(vertices), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
        # for i in range(0, len(center_list)):
        #     cv2.putText(output_img, "Center: " + str(center_list[i]), (0, 15 + i*15), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255))

        cv2.putText(output_img, "dis: " + str(getDistance(center_list)), (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))



        output_stream.putFrame(output_img)
        time.sleep(processing_time)
