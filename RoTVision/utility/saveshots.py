# Saves left and right stereo image pair

import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import sys
import os
import time

__author__ = "Shaun Price"
__date__ = "10 March 2018"

os.system('sudo modprobe bcm2835-v4l2')

patternsize = (9, 6)
captures = 0
cornersLeft = 0
cornersRight = 0
width = 640
height = 480

img_size = (width,height)

# Camera 1
camera1 = PiCamera(camera_num=0, led_pin=2) # led_pin because it can't be detected for CM3)
camera1.resolution = img_size

# Camera 2
camera2 = PiCamera(camera_num=1, led_pin=30) # led_pin because it can't be detected for CM3
camera2.resolution = img_size

for n in range(50):
	print "Capture "+str(n+1)
	
	print "Capture in 3"
	time.sleep(1)
	print "Capture in 2"
	time.sleep(1)
	print "Capture in 1"
	time.sleep(1)
	print "Capturing..."

	rawCapture1 = PiRGBArray(camera1)
	rawCapture2 = PiRGBArray(camera2)

	camera1.capture(rawCapture1, format="bgr")
	camera2.capture(rawCapture2, format="bgr")

	frame_left = rawCapture2.array
	frame_right = rawCapture1.array
	
	patternLeftFound = cv2.findChessboardCorners(cv2.cvtColor(frame_left, cv2.COLOR_BGR2GRAY), patternsize)
	patternRightFound = cv2.findChessboardCorners(cv2.cvtColor(frame_right, cv2.COLOR_BGR2GRAY), patternsize)

	print "Left Found: "+str(patternLeftFound[0])
	print "Right Found: "+str(patternRightFound[0])

	if patternLeftFound[0] and patternRightFound[0]: 
		print "OK: Checkerboard found!"
		captures += 1
		cv2.imwrite("left/left-"+str(captures)+".jpg", frame_left)
		cv2.imwrite("right/right-"+str(captures)+".jpg", frame_right)    
		print "Saving images "+str(captures)
		time.sleep(2)
	else:
		print "ERROR: No checkerboard!"
	
print "Files saved"




