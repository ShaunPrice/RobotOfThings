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
import roslib
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
import rosgraph

################################################################
# Initialise the Raspberry Pi video
################################################################

################################################################
# Either run the following command before running the script:
#   sudo modprobe bcm2835-v4l2
# or add bcm2835-v4l2 to the /etc/modules file
################################################################

################################################################
# Image Parameters
################################################################

width = 640
height = 480
crop = 50
img_size = (width,height)

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
    def __init__(self, resolution=(width, height*2), framerate=32):
        # initialize the camera and stream
        self.camera = PiCamera(stereo_mode="top-bottom")
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
        self.camera_pub = rospy.Publisher("camera",Image,queue_size=20)
        self.bridge = CvBridge()

        # keep looping infinitely until the thread is stopped
        for frame in self.stream:
            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = frame.array

            # Take the left image and send it as the camera image
            camera = cv2.rotate(self.frame[height:height*2-1,0:width],cv2.ROTATE_180)

            try:
                self.camera_pub.publish(self.bridge.cv2_to_imgmsg(camera, "bgr8"))
            except CvBridgeError as e:
                print(e)

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

        self.disparity_pub = rospy.Publisher("stereo/disparity",Image,queue_size=5)
        self.left_pub = rospy.Publisher("stereo/left_image",Image,queue_size=5)
        self.right_pub = rospy.Publisher("stereo/right_image",Image,queue_size=5)
        self.pointcloud_pub = rospy.Publisher("stereo/pointcloud",PointCloud2,queue_size=5)
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
        window_size = 5      # 5      # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
        min_disp = -48                # -48 - must be devisible by 16 
        num_disp = -min_disp-min_disp #  48*2 - must be devisible by 16

        self.left_matcher = cv2.StereoSGBM_create(
                minDisparity=min_disp,
                numDisparities=num_disp,       # max_disp has to be dividable by 16 f. E. HH 192, 256
                blockSize=5, # 5
                P1=8 * 3 * window_size ** 2,   # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
                P2=32 * 3 * window_size ** 2,
                disp12MaxDiff=-1,
                uniquenessRatio=1,  # 1
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

        self.wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=self.left_matcher)
        self.wls_filter.setLambda(lmbda)
        self.wls_filter.setSigmaColor(sigma)
        
        self.stopped = False

        print("Initialisation Complete")

    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        print("Starting Disparity Loop")
        # keep looping infinitely until the thread is stopped
        
        print("Start the capture thread")
        self.videoStream = PiVideoStream().start()

        print("Waiting")
        time.sleep(2)

        print("Starting Capture Loop")
        while True:
            image = np.empty((height*2 * width * 3), dtype=np.uint8)
            image = self.videoStream.read()

            try:
                ################################################################
                if image is not None:
                    greyImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
                    greyRight = greyImage[0:height-1,0:width]
                    greyLeft = greyImage[height:height*2-1,0:width]

                    time.sleep(0)

                    ################################################################
                    #Rectify the images:
                    leftImageRectified = cv2.remap(greyLeft, self.xmap1, self.ymap1, cv2.INTER_LINEAR)
                    rightImageRectified = cv2.remap(greyRight, self.xmap2, self.ymap2, cv2.INTER_LINEAR)

                    leftImageRectifiedRotated = cv2.rotate(leftImageRectified, cv2.ROTATE_180)
                    rightImageRectifiedRotated = cv2.rotate(rightImageRectified, cv2.ROTATE_180)
                    
                    ################################################################
                    # Now we can compute the disparities and convert the resulting images to the desired int16 format or how OpenCV names it: CV_16S for our filter:
                    displ = self.left_matcher.compute(leftImageRectifiedRotated, rightImageRectifiedRotated)  # .astype(np.float32)/16
                    dispr = self.right_matcher.compute(rightImageRectifiedRotated, leftImageRectifiedRotated)  # .astype(np.float32)/16

                    displ = np.int16(displ)
                    dispr = np.int16(dispr)
                    filteredImg = self.wls_filter.filter(dispr, leftImageRectifiedRotated, None, dispr)  # important to put "leftImageRectifiedRotated" here!!!

                    ################################################################
                    # Finally if you show this image with imshow() you may not see anything. This is due to values being not normalized to a 8-bit format. So lets fix this by normalizing our depth map:
                    cv2.normalize(src=filteredImg, dst=filteredImg, alpha=0, beta=255,  norm_type=cv2.NORM_MINMAX)

                    cropDispLeft = width - crop
                    cropDispRight = crop
                    # Note that the image is rotated 180 at this point
                    filteredImg = filteredImg[0:height, cropDispRight:cropDispLeft]
                    filteredImg = np.uint8(filteredImg)
                    disparityImg = cv2.rotate(filteredImg,cv2.ROTATE_180)

                    ################################################################
                    # Send Disparity Image
                    try:
                        self.disparity_pub.publish(self.bridge.cv2_to_imgmsg(disparityImg, "mono8"))
                    except CvBridgeError as e:
                        print(e)
                    
                    # Send Rectified Left Greyscale Image
                    try:
                        self.left_pub.publish(self.bridge.cv2_to_imgmsg(leftImageRectified, "mono8"))
                    except CvBridgeError as e:
                        print(e)

                    # Send Rectified Right Greyscale Image
                    try:
                        self.right_pub.publish(self.bridge.cv2_to_imgmsg(rightImageRectified, "mono8"))
                    except CvBridgeError as e:
                        print(e)

                    # Create the pointcloud
                    pointcloudImg = np.rot90(disparityImg) 
                    points = cv2.reprojectImageTo3D(pointcloudImg,self.Q)
                    pointsOut = points.reshape((-1,3))
                    pointsOut = pointsOut / 8 # Convert to meters

                    header = Header()
                    header.stamp = rospy.Time.now()
                    header.frame_id = "stereo/pointcloud"

                    pointCloud = point_cloud2.create_cloud_xyz32(header, pointsOut)

                    # Send Point Cloud
                    try:
                        self.pointcloud_pub.publish(pointCloud)
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
            # Catch when an image error is thrown
            except AttributeError as e:
                print(e)   
                raise e  
            except KeyboardInterrupt as ki:
                print("Exiting Loop")
                raise ki

    def stop(self):
        print("Closing Camera Stream")
        self.videoStream.stop()

        # indicate that the thread should be stopped
        self.stopped = True

################################################################

def main(args):
    print("Start the disparity thread")
    disp = Disparity().start()
    
    rospy.init_node('Disparity', anonymous=True)

    rospy.spin()
    
    print("Stopping")
    print("Closing Disparity")
    disp.stop()
    
    print("Stopped")
    os._exit(0)

if __name__ == '__main__':
    main(sys.argv)
################################################################
