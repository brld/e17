# Imports

import csv
import cv2
import cv2.aruco as aruco
import numpy as np
import math
import os
import shlex
import subprocess
import tempfile

import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.Piloting import moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from time import sleep

class Stream:

    def __init__(self):

        # Initalize the olympe.Drone object from its IP address
        self.drone = olympe.Drone(
            "192.168.42.1",
            loglevel=0,
        )
        self.tempd = tempfile.mkdtemp(prefix="olympe_streaming_test_")
        print("Olympe streaming example output dir: {}".format(self.tempd))
        self.h264_frame_stats = []
        self.h264_stats_file = open(
            os.path.join(self.tempd, 'h264_stats.csv'), 'w+')
        self.h264_stats_writer = csv.DictWriter(
            self.h264_stats_file, ['fps', 'bitrate'])
        self.h264_stats_writer.writeheader()

    def start(self):
        # Establish a connection the the drone
        self.drone.connection()

        self.drone.set_streaming_output_files(
            h264_data_file=os.path.join(self.tempd, 'h264_data.264'),
            h264_meta_file=os.path.join(self.tempd, 'h264_metadata.json'),
        )

        # Setup callback functions to perform live video processing
        self.drone.set_streaming_callbacks(
            raw_cb=self.yuv_frame_cb,
            h264_cb=self.h264_frame_cb
        )

        # Begin video streaming
        self.drone.start_video_streaming()
        self.drone(
            TakeOff()
            >> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait()

    def navigate(self):
        self.reachedDest = False
        while not self.reachedDest:
            if not self.corners:
                self.drone(moveBy(0, 0, 0, 0.5, _timeout=5)).wait().success()
            else:
                for corner in self.corners:
                    offset = False
                    x, y, width, height = cv2.boundingRect(corner)

                    vert = 0
                    hor = 0
                    dist = 0

                    top_dist = y
                    left_dist = x
                    bottom_dist = self.cv2frame.shape[0] - y - height
                    right_dist = self.cv2frame.shape[1] - x - width

                    vert_dist = top_dist - bottom_dist
                    hor_dist = left_dist - right_dist

                    threshold = cv2.contourArea(corner)

                    if offset:
                        self.drone(moveBy(dist * 0.25, hor * 0.1, vert * 0.1, 0, _timeout=5)).wait().success()
                        offset = False

                    if abs(top_bot_diff) >100:
                        if top_bot_diff>0:
                            offset = True
                            vert = 1
                            print('D-ADJUST')
                        else:
                            offset = True
                            vert = -1
                            print('U-ADJUST')

                    if abs(left_right_diff) >150:
                        if left_right_diff>0:
                            offset = True
                            hor = 1
                            print('R-ADJUST')
                        else:
                            offset = True
                            hor = -1
                            print('L-ADJUST')

                    if threshold < self.cv2frame.shape[0] * self.cv2frame.shape[1] * 0.065:
                        offset = True
                        dist = 1
                        print('F-ADJUST')
                    else:
                        self.reachedDest = True
                        break

    def stop(self):
        # Disconnect both the video stream and the drone
        self.drone(Landing()).wait()
        self.drone.stop_video_streaming()
        self.drone.disconnection()
        self.h264_stats_file.close()

    def yuv_frame_cb(self, yuv_frame):

        info = yuv_frame.info()
        height, width = info["yuv"]["height"], info["yuv"]["width"]

        # Convert PDRAW YUV flag to an OpenCV YUV flag
        cv2_cvt_color_flag = {
            olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
        }[info["yuv"]["format"]]

        # Utilize OpenCV to convert the YUV frame to RGB
        self.cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)

        gray = self.cv2frame
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        parameters =  aruco.DetectorParameters_create()

        self.corners, self.ids, self.rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        gray = aruco.drawDetectedMarkers(gray, self.corners)

        # Use OpenCV to display this frame
        cv2.imshow("Test Frame", gray)
        cv2.waitKey(1)

    def h264_frame_cb(self, h264_frame):
        """
        This function will be called by Olympe for each new h264 frame.
            :type yuv_frame: olympe.VideoFrame
        """

        # Get a ctypes pointer and size for this h264 frame
        frame_pointer, frame_size = h264_frame.as_ctypes_pointer()

        # Compute some stats and dump them in a csv file
        info = h264_frame.info()
        frame_ts = info["ntp_raw_timestamp"]
        if not bool(info["h264"]["is_sync"]):
            if len(self.h264_frame_stats) > 0:
                while True:
                    start_ts, _ = self.h264_frame_stats[0]
                    if (start_ts + 1e6) < frame_ts:
                        self.h264_frame_stats.pop(0)
                    else:
                        break
            self.h264_frame_stats.append((frame_ts, frame_size))
            h264_fps = len(self.h264_frame_stats)
            h264_bitrate = (
                8 * sum(map(lambda t: t[1], self.h264_frame_stats)))
            self.h264_stats_writer.writerow(
                {'fps': h264_fps, 'bitrate': h264_bitrate})

if __name__ == "__main__":
    # Instantiate stream object
    stream = Stream()

    # Begin script
    stream.start()

    # Proceed with navigation method
    stream.navigate()

    # Stop video streaming and commence landing
    stream.stop()
