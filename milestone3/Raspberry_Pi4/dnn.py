#!/usr/bin/env python 
from __future__ import division

# Imports
import tensorflow as tf
model = __import__("model")
import cv2
import sys
import os
import time
import math
import numpy as np
import serial
import picamera
import picamera.array


# Radian <-> Degree conversion functions
def deg2rad(deg):
        return deg * math.pi / 180.0
def rad2deg(rad):
        return 180.0 * rad / math.pi

#Get and set the number of cores to be used by TensorFlow
if(len(sys.argv) > 1):
	NCPU = int(sys.argv[1])
else:
	NCPU = 1
config = tf.ConfigProto(intra_op_parallelism_threads=NCPU, \
                        inter_op_parallelism_threads=NCPU, \
                        allow_soft_placement=True, \
                        device_count = {'CPU': 1})

#The max number of frames to be processed, 
#    and the number of frames already processed
NFRAMES = 1000
curFrame = 0

#Periodic task options
period = 50
is_periodic = True

#Load the model
sess = tf.InteractiveSession(config=config)
saver = tf.train.Saver()
model_load_path = "model/model.ckpt"
saver.restore(sess, model_load_path)

#Create lists for tracking operation timings
cap_time_list = []
prep_time_list = []
pred_time_list = []
tot_time_list = []

#Initialize serial connection to HiFive board
ser1 = serial.Serial('/dev/ttyAMA1', 115200)

print('---------- Processing video for epoch 1 ----------')

#Open the video file
#vid_path = 'epoch-1.avi'
#assert os.path.isfile(vid_path)
#cap = cv2.VideoCapture(vid_path)

#Process the video while recording the operation execution times
print('performing inference... press q to quit')
time_start = time.time()
first_frame = True
with picamera.PiCamera() as camera:
    with picamera.array.PiRGBArray(camera, (640,480)) as rawCapture:
        camera.resolution = (640,480)
        camera.framerate = 30
        #Allow camera to warm up
        time.sleep(0.1)
        
        for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
            image = frame.array
            img = cv2.resize(image, (200,66))
            img = img/255.            
    
    #Feed the frame to the model and get the control output
            rad = model.y.eval(feed_dict={model.x: [img]})[0][0]
            deg = rad2deg(rad)

            cv2.imshow('Frame', image)
            key = cv2.waitKey(1) & 0xFF
            ser1.write("angle:"+str(int(deg))+"\n")
            
            rawCapture.truncate(0)
            
            if key == ord('q'):
                break       

ser1.close() #close serial connection to HiFive

#Calculate and output FPS/frequency
fps = curFrame / (time.time() - time_start)
print('completed inference, total frames: {}, average fps: {} Hz'.format(curFrame+1, round(fps, 1)))

#Calculate and display statistics of the total inferencing times
print("count: {}".format(len(tot_time_list)))
print("mean: {}".format(np.mean(tot_time_list)))
print("max: {}".format(np.max(tot_time_list)))
print("99.999pct: {}".format(np.percentile(tot_time_list, 99.999)))
print("99.99pct: {}".format(np.percentile(tot_time_list, 99.99)))
print("99.9pct: {}".format(np.percentile(tot_time_list, 99.9)))
print("99pct: {}".format(np.percentile(tot_time_list, 99)))
print("min: {}".format(np.min(tot_time_list)))
print("median: {}".format(np.median(tot_time_list)))
print("stdev: {}".format(np.std(tot_time_list)))
