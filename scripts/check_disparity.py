#!/opt/anaconda3/bin/python
import numpy as np
import cv2

def zoom_in_by_cropping(frame, scale = 40, final_shape =(640,480)):
    #get the webcam size
    height, width, channels = frame.shape

    #prepare the crop
    centerX,centerY=int(height/2),int(width/2)
    radiusX,radiusY= int(scale*height/100),int(scale*width/100)

    minX,maxX=centerX-radiusX,centerX+radiusX
    minY,maxY=centerY-radiusY,centerY+radiusY

    cropped = frame[minX:maxX, minY:maxY]
    resized_cropped = cv2.resize(cropped, (final_shape[0], final_shape[1])) 
    return resized_cropped

def get_correspondece(frame0, frame1):
    gray_l = cv2.cvtColor(frame0,cv2.COLOR_RGB2GRAY)
    gray_r = cv2.cvtColor(frame1,cv2.COLOR_RGB2GRAY)
    sift = cv2.SIFT_create()

    # Detect keypoints and compute descriptors
    kp0, des0 = sift.detectAndCompute(gray_l,None)
    kp1, des1 = sift.detectAndCompute(gray_r,None)

    # create BFMatcher object
    bf = cv2.BFMatcher()
    # Match descriptors.
    matches = bf.knnMatch(des0,des1,k=2)
    # Apply ratio test
    threshold = 0.85 # tweak treshhold to better results
    good = []
    for m,n in matches:
        if m.distance < threshold*n.distance:
            good.append([m])
    print("Good matches:",len(good))
    # Draw first 10 matches.
    img3 = cv2.drawMatchesKnn(frame0,kp0,frame1,kp1,good[:10],None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    return img3

def get_disparity(frame0, frame1):
    gray_l = cv2.cvtColor(frame0,cv2.COLOR_RGB2GRAY)
    gray_r = cv2.cvtColor(frame1,cv2.COLOR_RGB2GRAY)
    disparity =  stereo.compute(gray_l, gray_r).astype(np.float32) / 16.0
    disparity_normalized = cv2.normalize(src=disparity,
                                     dst=None,
                                     beta=0,
                                     alpha=255,
                                     norm_type=cv2.NORM_MINMAX)
    # Convert to 8-bit image
    disparity_normalized = np.uint8(disparity_normalized)
    return disparity_normalized

stereo = cv2.StereoSGBM_create()

def on_trackbar_minDisparity(val):
    stereo.setMinDisparity(val)
    im_h = get_disparity(zoom_in_by_cropping(frame0_dst), frame1_dst)
    cv2.imshow('disparity', im_h)

def on_trackbar_numDisparities(val):
    stereo.setNumDisparities(val*16)
    im_h = get_disparity(zoom_in_by_cropping(frame0_dst), frame1_dst)
    cv2.imshow('disparity', im_h)

def on_trackbar_blocksize(val):
    stereo.setBlockSize(val)
    im_h = get_disparity(zoom_in_by_cropping(frame0_dst), frame1_dst)
    cv2.imshow('disparity', im_h)

def on_trackbar_P1(val):
    stereo.setP1(8 * 3 * val ** 2)
    im_h = get_disparity(zoom_in_by_cropping(frame0_dst), frame1_dst)
    cv2.imshow('disparity', im_h)

def on_trackbar_P2(val):
    stereo.setP2(32 * 3 * val ** 2)
    im_h = get_disparity(zoom_in_by_cropping(frame0_dst), frame1_dst)
    cv2.imshow('disparity', im_h)

def on_trackbar_disp12MaxDiff(val):
    stereo.setDisp12MaxDiff(val)
    im_h = get_disparity(zoom_in_by_cropping(frame0_dst), frame1_dst)
    cv2.imshow('disparity', im_h)

def on_trackbar_uniquenessRatio(val):
    stereo.setUniquenessRatio(val)
    im_h = get_disparity(zoom_in_by_cropping(frame0_dst), frame1_dst)
    cv2.imshow('disparity', im_h)

def on_trackbar_speckleWindowSize(val):
    stereo.setSpeckleWindowSize(val)
    im_h = get_disparity(zoom_in_by_cropping(frame0_dst), frame1_dst)
    cv2.imshow('disparity', im_h)

def on_trackbar_speckleRange(val):
    stereo.setSpeckleRange(val)
    im_h = get_disparity(zoom_in_by_cropping(frame0_dst), frame1_dst)
    cv2.imshow('disparity', im_h)

matrix0 = np.loadtxt('calib_imgs/calib_photos_DH_0918B/camera_matrix.npy')
distortion0 = np.loadtxt('calib_imgs/calib_photos_DH_0918B/distortion_coefficient.npy')

matrix1 = np.loadtxt('calib_imgs/calib_photos_PC_CAM_OV1320_V1.1/camera_matrix.npy')
distortion1 = np.loadtxt('calib_imgs/calib_photos_PC_CAM_OV1320_V1.1/distortion_coefficient.npy')

cam0 = cv2.VideoCapture(0)
cam0.set(3,800) # 800 -20% 800 = 640
cam0.set(4,600) # 600 - 20% 600 = 480
cam1 = cv2.VideoCapture(1)
cam1.set(3,640)
cam1.set(4,480)

rval, frame0_dst = cam0.read()
rval, frame1_dst = cam1.read()

cv2.namedWindow('disparity')
cv2.createTrackbar('setMinDisparity', 'disparity' , 1, 36, on_trackbar_minDisparity) 
cv2.createTrackbar('setNumDisparities x16', 'disparity' , 1, 10, on_trackbar_numDisparities)
cv2.createTrackbar('setBlockSize', 'disparity' , 4, 11, on_trackbar_blocksize) 
cv2.createTrackbar('setP1', 'disparity' , 0, 20, on_trackbar_P1) 
cv2.createTrackbar('setP2', 'disparity' , 0, 20, on_trackbar_P2) 
cv2.createTrackbar('setDisp12MaxDiff', 'disparity' , 20, 128, on_trackbar_disp12MaxDiff)
cv2.createTrackbar('setUniquenessRatio)', 'disparity' , 5, 25, on_trackbar_uniquenessRatio)
cv2.createTrackbar('setSpeckleWindowSize', 'disparity' , 200, 400, on_trackbar_speckleWindowSize)
cv2.createTrackbar('setSpeckleRange', 'disparity' , 1, 2, on_trackbar_speckleRange)

while True:
    rval0, frame0 = cam0.read()
    rval1, frame1 = cam1.read()
    if rval0 and rval1:
        # horizontally concatenates images of same height  
        im_h = cv2.hconcat([zoom_in_by_cropping(frame0 ), frame1]) 
        cv2.imshow("cam 0", im_h)
        # sift correspondences
        #im_h = get_correspondece(zoom_in_by_cropping(frame0), frame1)
        # undistorted
        frame0_dst = cv2.undistort(frame0, matrix0, distortion0, None)
        im_v = cv2.hconcat([zoom_in_by_cropping(frame0_dst) , zoom_in_by_cropping(frame0)]) 
        frame1_dst = cv2.undistort(frame1, matrix1, distortion1, None)
        im_h = cv2.hconcat([frame1_dst , frame1]) 
        vh = cv2.vconcat([im_v , im_h])
        #cv2.imshow("cam 0", vh)
    if cv2.waitKey(20) == 27:  # exit on ESC
        break

cv2.destroyAllWindows() 