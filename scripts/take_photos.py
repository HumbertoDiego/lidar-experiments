#!/usr/bin/python3
import cv2
import time
import argparse
import os

BASE_PHOTO_PATH = '.'
SKIP_FIRST_FRAMES = 20

def list_avaiable_cameras():
    a = []
    for port in range(1,2):
        cam = cv2.VideoCapture(port)
        if not cam.isOpened():
            print(f"Camera {port} is not opened.")
        else:
            a.append(port)
        cam.release()
    return a

def main(t):
    folder_name = len(os.listdir(BASE_PHOTO_PATH))+1
    path = f"{BASE_PHOTO_PATH}/{folder_name}"
    os.mkdir(path)

    cam = []
    avaiable_cameras = [1]#list_avaiable_cameras()
    for k in avaiable_cameras:
        cam.append(cv2.VideoCapture(k,cv2.CAP_DSHOW))
    
    # Skip first frames as it is still warming up
    count = 0
    while count < SKIP_FIRST_FRAMES:
        for i in range(len(cam)):
            ret, image = cam[i].read()
        count+=1
    
    cam[0].set(cv2.CAP_PROP_GAMMA,255)
    current_exposure = cam[0].get(cv2.CAP_PROP_GAMMA)
    print(current_exposure)
    # cam[0].set(cv2.CAP_PROP_BRIGHTNESS, 50)
    # current_brightness = cam[0].get(cv2.CAP_PROP_BRIGHTNESS)
    # print(current_brightness)

    while True:
        ret, image = cam[0].read()
        frame = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX)
        cv2.imshow("Video", frame)

        if cv2.waitKey(1) == ord('q'):
            break
    # count = 0
    # t_end = time.time() + t
    # while time.time() < t_end:
    #     for i in range(len(cam)):
    #         ret, image = cam[i].read()
    #         if ret:
    #             pic_name = f'{path}/cam{i}_frame{count}.jpg'
    #             print(pic_name)
    #             cv2.imwrite(pic_name, image)
    #     count+=1
    
    # for i in range(len(cam)):
    #     cam[i].release()
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-s','--segundos',
                        default=5,
                        type=int,
                        help='Add time in seconds')
    args = parser.parse_args()
    print(f"Starting taking {args.segundos} seconds of photos.")
    main(args.segundos)

"""
User Controls

                     brightness 0x00980900 (int)    : min=0 max=255 step=1 default=128 value=128
                       contrast 0x00980901 (int)    : min=0 max=255 step=1 default=128 value=128
                     saturation 0x00980902 (int)    : min=0 max=255 step=1 default=128 value=128
                            hue 0x00980903 (int)    : min=-256 max=-32513 step=1 default=-32640 value=128
        white_balance_automatic 0x0098090c (bool)   : default=0 value=0
                    red_balance 0x0098090e (int)    : min=0 max=255 step=0 default=128 value=128
                   blue_balance 0x0098090f (int)    : min=0 max=255 step=1 default=128 value=128
                          gamma 0x00980910 (int)    : min=0 max=255 step=10 default=128 value=128
                           gain 0x00980913 (int)    : min=0 max=255 step=1 default=128 value=128
           power_line_frequency 0x00980918 (menu)   : min=0 max=2 default=0 value=0
				0: Disabled
				1: 50 Hz
				2: 60 Hz
                  hue_automatic 0x00980919 (bool)   : default=0 value=0
      white_balance_temperature 0x0098091a (int)    : min=0 max=255 step=1 default=128 value=128
                      sharpness 0x0098091b (int)    : min=0 max=255 step=1 default=128 value=128
         backlight_compensation 0x0098091c (int)    : min=0 max=255 step=1 default=128 value=128

Camera Controls

                  auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=0 value=0
         exposure_time_absolute 0x009a0902 (int)    : min=0 max=0 step=0 default=0 value=0 flags=inactive
     exposure_dynamic_framerate 0x009a0903 (bool)   : default=0 value=0
                   pan_absolute 0x009a0908 (int)    : min=0 max=0 step=0 default=0 value=0
                  tilt_absolute 0x009a0909 (int)    : min=0 max=0 step=0 default=0 value=0
                 focus_absolute 0x009a090a (int)    : min=0 max=0 step=0 default=0 value=0
     focus_automatic_continuous 0x009a090c (bool)   : default=0 value=0
                  zoom_absolute 0x009a090d (int)    : min=0 max=0 step=0 default=0 value=0
                zoom_continuous 0x009a090f (int)    : min=0 max=0 step=0 default=0 value=0 flags=write-only
                        privacy 0x009a0910 (bool)   : default=0 value=0
                  iris_absolute 0x009a0911 (int)    : min=0 max=0 step=0 default=0 value=0
                  iris_relative 0x009a0912 (int)    : min=0 max=0 step=0 default=0 value=0 flags=write-only
                      pan_speed 0x009a0920 (int)    : min=0 max=0 step=0 default=0 value=0
                     tilt_speed 0x009a0921 (int)    : min=0 max=0 step=0 default=0 value=0

"""