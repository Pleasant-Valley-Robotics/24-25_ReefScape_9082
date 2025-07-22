#!/usr/bin/env python3

from cscore import CameraServer
from ntcore import NetworkTableInstance, EventFlags
from cscore import UsbCamera
import cv2
import json
import numpy as np
import time

import robotpy_apriltag
from wpimath.geometry import Transform3d
import math

team = 9082
server = False


def main():
    with open('/boot/frc.json') as f:
        config = json.load(f)
    camera = config['cameras'][0]
    width = camera['width']
    height = camera['height']

    #Start detecting the logitechC920 camera on port 0
    logitechC920 = CameraServer.startAutomaticCapture(0)
    logitechC920.setFPS(20)
    logitechC920.setResolution(320,240)

    # Setup camera streams for processing video
    input_stream = CameraServer.getVideo()
    output_stream = CameraServer.putVideo('Processed', width, height)
    img = np.zeros(shape=(height, width, 3), dtype=np.uint8)

    # start NetworkTables
    ntinst = NetworkTableInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClient4("wpilibpi")
        ntinst.setServerTeam(team)
        ntinst.startDSClient()

    # Wait for NetworkTables to start
    time.sleep(0.5)

    prev_time = time.time()
    while True:
        start_time = time.time()

        frame_time, input_img = input_stream.grabFrame(img)
        output_img = np.copy(input_img)
        output_img = cv2.line(output_img, (200,0),(200, 120),(255,0,0),5)
        output_img = cv2.line(output_img, (138,0), (138, 120), (255,0,0),5)
        output_img = cv2.flip(cv2.flip(output_img,0),1)
        
        # Notify output of error and skip iteration 	
        if frame_time == 0:
            output_stream.notifyError(input_stream.getError())
            continue

        processing_time = start_time - prev_time
        prev_time = start_time
    
        fps = 1 / processing_time
        cv2.putText(output_img, str(round(fps, 1)), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
        output_stream.putFrame(output_img)
        #output_stream.setResolution(1280,720)


main()
