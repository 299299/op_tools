#!/usr/bin/python3

import cv2
import os
import sys
import rlog_reader as lr
import time
import numpy as np

from ui_helpers import (_BB_TO_FULL_FRAME, _INTRINSICS, Calibration, plot_model, plot_nav)

width = 1164
height = 874
num_px = width * height
intrinsic_matrix = _INTRINSICS[num_px]
zoom_matrix = _BB_TO_FULL_FRAME[num_px]

def video_test(video_file):
    cap = cv2.VideoCapture(video_file)
    frame_count = 1
    ret = 1
    while(ret):
        ret, frame = cap.read()
        #print (frame_count)
        if ret:
            frame_count = frame_count + 1
    cap.release()

    print (frame_count)

class VideoReader():
    def __init__(self):
        pass

    def open(self, video_file):
        self._cap = cv2.VideoCapture(video_file)
        self._frame = 0
        self._last_img = None
        # self._total_frames = self._cap.get(cv2.CV_CAP_PROP_FRAME_COUNT)
        # print ('video toltal_frames=' + str(self._total_frames))

    def close(self):
        self._cap.release()

    def get_img(self, target_frame):
        #print ('video --> ' + str(target_frame))
        #self._cap.set(cv2.CAP_PROP_POS_FRAMES, target_frame)
        self._frame += 1
        success, image = self._cap.read()
        # print (success)

        if success:
            self._last_img = image
        else:
            image = self._last_img

        return image

class SegmentReader():
    def __init__(self):
        self._log_reader = None
        self._video_reader = VideoReader()
        self._events = []
        self._start_ts = 0
        self._img = np.zeros((480, 640, 3), dtype='uint8')
        self._model = None
        self._nav = None


    def open(self, path):
        rlog_file = path + '/rlog.bz2'
        fcam_video = path + '/fcamera.hevc'

        self._log_reader = lr.LogReader(rlog_file)
        self._video_reader.open(fcam_video)

        topic_for_events = ['roadEncodeIdx', 'sensorEvents', 'navInstruction',
                             'gpsLocationExternal', 'cameraOdometry',
                             'modelV2', 'liveCalibration']
        logs = list(self._log_reader)

        for i in logs:
            if i.which in topic_for_events:
                self._events.append(i)
                if self._start_ts == 0:
                    self._start_ts = i.logMonoTime

            # if i.which == 'cameraOdometry':
            #     print (i)

        print ("start mono time=" + str(self._start_ts))


    def processVideo(self, evt):
        img = self._video_reader.get_img(evt.roadEncodeIdx.encodeId)

        #print (zoom_matrix)

        cv2.warpAffine(img, zoom_matrix[:2], (self._img.shape[1], self._img.shape[0]), dst=self._img, flags=cv2.WARP_INVERSE_MAP)
        # self._img = img

    def processSensorEvents(self, evt):
        for se in evt.sensorEvents:
            if se.sensor == 1: # accleration
                #print (se)
                acceleration = se.acceleration.v
                #print (acceleration)

    def processNav(self, evt):
        #print (evt)
        self._nav = evt.navInstruction

    def processModel(self, evt):
        self._model = evt.modelV2

    def processCalibration(self, evt):
        rpyCalib = np.asarray(evt.liveCalibration.rpyCalib)
        self._calibration = Calibration(num_px, rpyCalib, intrinsic_matrix)

    def loop(self):

        replay_start_ts = self._start_ts
        real_time_start_ts = time.time_ns()
        num_evt = len(self._events)
        index = 0

        while index < num_evt:
            evt = self._events[index]

            event_ts = evt.logMonoTime
            realt_time_ts = time.time_ns()

            event_dt = event_ts - replay_start_ts
            real_time_dt = realt_time_ts - real_time_start_ts

            image_updated = False

            #print (evt)
            #print ('event_dt=' + str(event_dt) + ' real_time_dt=' + str(real_time_dt))

            if event_dt >= real_time_dt:
                time.sleep(0.001) # check every 1 ms
                continue

            if evt.which == 'roadEncodeIdx':
                self.processVideo(evt)
                image_updated = True

            if evt.which == 'sensorEvents':
                self.processSensorEvents(evt)

            if evt.which == 'navInstruction':
                self.processNav(evt)

            if evt.which == 'modelV2':
                self.processModel(evt)

            if evt.which == 'liveCalibration':
                self.processCalibration(evt)


            if image_updated:
                if self._model:
                    # print (self._model)
                    plot_model(self._model, self._img, self._calibration)
                if self._nav:
                    plot_nav(self._nav, self._img)

                cv2.imshow('road_camera', self._img)
                cv2.waitKey(1)

            index += 1



    def close(self):
        self._video_reader.close()
        del self._log_reader





if __name__ == "__main__":
    folder_path = sys.argv[1]

    segment_reader = SegmentReader()
    segment_reader.open(folder_path)

    segment_reader.loop()

    segment_reader.close()




