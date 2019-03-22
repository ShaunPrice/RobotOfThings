# Saves left and right stereo image pair

import cv2
import sys
import os

__author__ = "Shaun Price"
__date__ = "10 March 2018"

os.system('sudo modprobe bcm2835-v4l2')

cap_left = cv2.VideoCapture(1)
cap_left.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

cap_right = cv2.VideoCapture(0)
cap_right.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

ret_left, frame_left = cap_left.read()
ret_right, frame_right = cap_right.read()

str("Saving images")
cv2.imwrite("left.jpg", frame_left)
cv2.imwrite("right.jpg", frame_right)    

cv2.imshow('camera left', frame_left)
cv2.imshow('camera right', frame_right)

key = cv2.waitKey(0)

cap_left.release()
cap_right.release()
cv2.destroyAllWindows()

str("Files saved")




