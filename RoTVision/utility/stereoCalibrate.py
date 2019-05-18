import cv2
import glob
import numpy as np
import sys
import argparse

PATTERN_SIZE = (9, 6)
left_imgs = list(sorted(glob.glob('left/left*.jpg')))
right_imgs = list(sorted(glob.glob('right/right*.jpg')))
assert len(left_imgs) == len(right_imgs)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
left_pts, right_pts = [], []
img_size = None

for left_img_path, right_img_path in zip(left_imgs, right_imgs):
    left_img = cv2.imread(left_img_path, cv2.IMREAD_GRAYSCALE)
    right_img = cv2.imread(right_img_path, cv2.IMREAD_GRAYSCALE)

    #print('Left Image:  ')
    #print(left_img)
    #print('Right Image: ')
    #print(right_img)

    if img_size is None:
        img_size = (left_img.shape[1], left_img.shape[0])
    
    res_left, corners_left = cv2.findChessboardCorners(left_img, PATTERN_SIZE)
    res_right, corners_right = cv2.findChessboardCorners(right_img, PATTERN_SIZE)
    
    corners_left = cv2.cornerSubPix(left_img, corners_left, (10, 10), (-1,-1),
                                    criteria)
    corners_right = cv2.cornerSubPix(right_img, corners_right, (10, 10), (-1,-1), 
                                     criteria)
    
    left_pts.append(corners_left)
    right_pts.append(corners_right)

pattern_points = np.zeros((np.prod(PATTERN_SIZE), 3), np.float32)
pattern_points[:, :2] = np.indices(PATTERN_SIZE).T.reshape(-1, 2)
pattern_points = [pattern_points] * len(left_imgs)

err, Kl, Dl, Kr, Dr, R, T, E, F = cv2.stereoCalibrate(pattern_points, left_pts, right_pts, None, None, None, None, img_size, flags=0)

### Note that the left and right cameras are swapped to match the output from StereoVision
print('Left camera:')
print(Kl)
np.save('calibration/cam_mats_right.npy', Kl)

print('Left camera distortion:')
print(Dl)
np.save('calibration/dist_coefs_right.npy', Dl)

print('Right camera:')
print(Kr)
np.save('calibration/cam_mats_left.npy', Kr)

print('Right camera distortion:')
print(Dr)
np.save('calibration/dist_coefs_left.npy', Dr)

print('Rotation matrix:')
print(R)
np.save('calibration/rot_mat.npy', R)

print('Translation:')
print(T)
np.save('calibration/trans_vec.npy', T)
