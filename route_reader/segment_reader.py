#!/usr/bin/python3

import cv2
import os
import sys
import rlog_reader as lr
import time

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
            cv2.imshow('image', image)
            cv2.waitKey(1)
        else:
            image = self._last_img

        return image


if __name__ == "__main__":
    folder_path = sys.argv[1]
    rlog_file = folder_path + '/rlog.bz2'
    fcam_video = folder_path + '/fcamera.hevc'

    video_reader = VideoReader()
    video_reader.open(fcam_video)

    topic_to_print = ['roadEncodeIdx', 'carState']
    lr = lr.LogReader(rlog_file)
    logs = list(lr)

    ts = 0

    # find initData
    for i in logs:
        if i.which == 'initData':
            print (i)
            break


    for i in logs:
        #print(i.which())
        if i.which == 'roadEncodeIdx':
            img = video_reader.get_img(i.roadEncodeIdx.encodeId)
            print (i)

        if i.which == 'carState':
            ts = i.logMonoTime
            ts_since_boot_sec = ts / 1e9
            print (ts_since_boot_sec)
            print (i)


    video_reader.close()


