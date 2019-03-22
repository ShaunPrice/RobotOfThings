#!/usr/bin/env python

import os
import io
import time
import sys
import cv2
import signal
import struct
from threading import Thread
import numpy as np
import picamera
from picamera.array import PiRGBArray
from picamera import PiCamera
import matplotlib
from sklearn.preprocessing import normalize
import roslib
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

################################################################
# Initialise the Raspberry Pi video
################################################################
os.system("sudo modprobe bcm2835-v4l2")

################################################################
# Image Parameters
################################################################

width = 640
height = 480

scaledWidth = 480
scaledHeight = 360

crop = 0

img_size = (width,height)
rateCounter = 0

################################################################
# Image Rectification Parameters
################################################################
mtxLeftCamera = np.array(
[[ 486.51278415, 0.0,       308.48359086 ],
 [   0.0,      486.4152203, 240.17983287  ],
 [   0.0,        0.0,          1.0000     ]])

mtxLeftCameraDist = np.array([[0.22176547, -0.57371284, 0.00480635, -0.00072598, 0.42312626]])

mtxRightCamera = np.array(
[[ 486.27314153, 0.0,         313.49083907 ],
 [   0.0,       484.76326808, 239.66692669 ],
 [   0.0,         0.0,          1.0        ]])

mtxRightCameraDist = np.array([[2.27908780e-01, -6.87854215e-01, 4.25129188e-04, -3.12014438e-04, 5.83601997e-01]])

mtxRotation = np.array(
[[  0.99725049,     0.01428428, -0.07271464 ],
 [ -0.01059108,     0.99864632,  0.05092495 ],
 [  0.07334363,    -0.0500148,   0.99605182 ]])

mtxTranslation = np.array(
[[  2.5967436  ],
 [ -0.04762659 ],
 [  0.01515725 ]])

# High speed camera class
################################################################
class PiVideoStream:
    def __init__(self, resolution=(width*2, height), framerate=32):
        # initialize the camera and stream
        self.camera = PiCamera(stereo_mode="side-by-side")
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.rawCapture = PiRGBArray(self.camera, size=resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True)

        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.stopped = False

    def start(self):
        # start the thread to read frames from the video stream
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        # keep looping infinitely until the thread is stopped
        for frame in self.stream:
            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = frame.array

            self.rawCapture.truncate(0)
            # if the thread indicator variable is set, stop the thread
            # and resource camera resources
            if self.stopped:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                return
    def read(self):
        # return the frame most recently read
        return self.frame
 
    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

###############################################################
class Disparity:
    def __init__(self):
        print("Initialise Disparity")
        self.stereoStream = io.BytesIO()

        ################################################################
        print("Initialise ROS Bridge")

        self.disparity_pub = rospy.Publisher("disparity_image",Image,queue_size=10)
        self.stereo_pub = rospy.Publisher("stereo_image",Image,queue_size=10)
        self.bridge = CvBridge()

        ################################################################
        print("Estimate Stereo Rectification Parameters")
        # Estimate the stereo rectification parameters:
        self.R1, self.R2, self.P1, self.P2, self.Q, self.validRoi1, self.validRoi2 = cv2.stereoRectify(
                mtxLeftCamera, mtxLeftCameraDist, 
                mtxRightCamera, mtxRightCameraDist,
                img_size,mtxRotation, mtxTranslation)

        #Prepare the stereo rectification transformation maps:
        self.xmap1, self.ymap1 = cv2.initUndistortRectifyMap(mtxLeftCamera, mtxLeftCameraDist, self.R1, mtxLeftCamera, img_size, cv2.CV_32FC1)
        self.xmap2, self.ymap2 = cv2.initUndistortRectifyMap(mtxRightCamera, mtxRightCameraDist, self.R2, mtxRightCamera, img_size, cv2.CV_32FC1)

        print("Creating output video stream")

        ################################################################
        print("Creating the Stereo Matcher")
        # SGBM Parameters -----------------
        window_size = 5      # 5           # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
        min_disp = -48                    # -48 for 320x240 and 240x180, -96 for 640x480
        num_disp = 48-min_disp             # 48 for 320x240 and 240x180, 96 for 640x480

        self.left_matcher = cv2.StereoSGBM_create(
                minDisparity=min_disp,
                numDisparities=num_disp,             # max_disp has to be dividable by 16 f. E. HH 192, 256
                blockSize=5, # 5
                P1=8 * 3 * window_size ** 2,    # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
                P2=32 * 3 * window_size ** 2,
                disp12MaxDiff=-1,
                uniquenessRatio=5,  # 1
                speckleWindowSize=0,
                speckleRange=2,
                preFilterCap=7,
                mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )

        # This leads us to define the right_matcher so we can use it for our filtering later. This is a simple one-liner:
        self.right_matcher = cv2.ximgproc.createRightMatcher(self.left_matcher)

        # To obtain hole free depth-images we can use the WLS-Filter. This filter also requires some parameters which are shown below:

        ################################################################
        print("Preparing the WLS filter")

        # FILTER Parameters
        lmbda = 16000
        sigma = 2.0   #1.2
        visual_multiplier = 1.0

        self.wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=self.left_matcher)
        self.wls_filter.setLambda(lmbda)
        self.wls_filter.setSigmaColor(sigma)
        
        self.stopped = False

        print("Initialisation Complete")

    def start(self, videoStream):
        self.videoStream = videoStream
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        print("Starting Disparity Loop")
        # keep looping infinitely until the thread is stopped
        
        while True:
            image = np.empty((height * width*2 * 3,), dtype=np.uint8)
            image = self.videoStream.read()

            ################################################################
            if image.size:
                greyImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
                greyLeft = greyImage[0:height,0:width-1]
                greyRight = greyImage[0:height,width:width*2-1]

                c, r = greyLeft.shape

                time.sleep(0)

                ################################################################
                #Rectify the images:
                leftImageRectified = cv2.remap(greyLeft, self.xmap1, self.ymap1, cv2.INTER_LINEAR)
                rightImageRectified = cv2.remap(greyRight, self.xmap2, self.ymap2, cv2.INTER_LINEAR)
                leftImageRectifiedResized = cv2.resize(leftImageRectified , (scaledWidth,scaledHeight))
                rightImageRectifiedResized = cv2.resize(rightImageRectified , (scaledWidth,scaledHeight))

                leftImageRectifiedResizedRotated = cv2.rotate(leftImageRectifiedResized, cv2.ROTATE_180)
                rightImageRectifiedResizedRotated = cv2.rotate(rightImageRectifiedResized, cv2.ROTATE_180)

                ################################################################
                # Now we can compute the disparities and convert the resulting images to the desired int16 format or how OpenCV names it: CV_16S for our filter:
                displ = self.left_matcher.compute(leftImageRectifiedResizedRotated, rightImageRectifiedResizedRotated)  # .astype(np.float32)/16
                dispr = self.right_matcher.compute(rightImageRectifiedResizedRotated, leftImageRectifiedResizedRotated)  # .astype(np.float32)/16

                displ = np.int16(displ)
                dispr = np.int16(dispr)
                filteredImg = self.wls_filter.filter(displ, leftImageRectifiedResizedRotated, None, dispr)  # important to put "leftImageRectifiedResizedRotated" here!!!

                ################################################################
                # Finally if you show this image with imshow() you may not see anything. This is due to values being not normalized to a 8-bit format. So lets fix this by normalizing our depth map:
                cv2.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX)

                filteredImg = filteredImg[0:height, crop:width-crop]

                filteredImg = np.uint8(filteredImg)
                filteredImg = cv2.rotate(filteredImg,cv2.ROTATE_180)

                try:
                    self.disparity_pub.publish(self.bridge.cv2_to_imgmsg(cv2.resize(filteredImg,(width-crop,height)), "mono8"))
                    #print("Disparity Image Sent")
                except CvBridgeError as e:
                    print(e)

                try:
                    # Had to rotate the image. Need to clean this up so image isn't rotated to start with.
                    self.stereo_pub.publish(self.bridge.cv2_to_imgmsg(cv2.resize(cv2.rotate(image, cv2.ROTATE_180),(width,height)), "bgr8"))
                    #print("Stereo Image Sent")
                except CvBridgeError as e:
                    print(e)

                ################################################################
                # if the thread indicator variable is set, stop the thread
                if self.stopped:
                    return
                else:
                    time.sleep(0)
            else:
                time.sleep(0.5)
                print("No image to process")
 
    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

################################################################
def main(args):
    
    print("Start the capture thread")
    videoStream = PiVideoStream().start()

    print("Waiting")
    time.sleep(2)

    print("Start the disparity thread")
    disp = Disparity().start(videoStream)
    
    rospy.init_node('Disparity', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Stopping")
        print("Closing Disparity")
        disp.stop()
        print("Closing Camera Stream")
        videoStream.stop()
        print("Stopped")
        os._exit(0)

if __name__ == '__main__':
    main(sys.argv)
################################################################
